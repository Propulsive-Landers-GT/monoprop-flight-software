//! Host test: the firmware's worst stack path must stay under half the configured stack.
//!
//! It builds the release firmware (in its own target directory, so it doesn't wait on the outer cargo),
//! disassembles it, and adds up each function's stack frame: every `sub sp` immediate plus the registers its
//! `push` / `vpush` save. The path summed is main + Navigator::step + Esekf::update::<3> + the largest frame
//! of any other function (standing in for whatever helper is deepest under update). The configured stack is
//! read from the ELF's `__stack_size`, and checked against TEENSY4_STACK_SIZE in .cargo/config.toml.
//! An unrecognised stack adjustment (e.g. `sub sp, rN`) fails the test rather than being ignored.
//! Needs a disassembler that reads ARM ELF: llvm-objdump, rust-objdump, or macOS's /usr/bin/objdump.
use std::collections::HashMap;
use std::path::PathBuf;
use std::process::Command;

fn monoprop_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn objdump() -> Option<String> {
    ["llvm-objdump", "rust-objdump", "/usr/bin/objdump"]
        .iter()
        .find(|tool| Command::new(tool).arg("--version").output().is_ok_and(|o| o.status.success()))
        .map(|t| t.to_string())
}

/// Bytes a `push`/`push.w`/`vpush` register list saves.
fn push_bytes(line: &str) -> u64 {
    let (Some(a), Some(b)) = (line.find('{'), line.find('}')) else { return 0 };
    let regs: Vec<&str> = line[a + 1..b].split(',').map(str::trim).filter(|r| !r.is_empty()).collect();
    let per = if line.contains("vpush") { 8 } else { 4 };
    regs.len() as u64 * per
}

/// The immediate of `sub sp, #N` / `sub.w sp, sp, #N`; Err for any other form that moves sp down.
fn sub_sp_bytes(line: &str) -> Result<u64, String> {
    let l = line.split('@').next().unwrap_or(line);
    let Some(rest) = l.split_once("sub").map(|(_, r)| r) else { return Ok(0) };
    let ops: Vec<&str> = rest.trim_start_matches(".w").split(',').map(str::trim).collect();
    if ops.first() != Some(&"sp") {
        return Ok(0);
    }
    let imm = ops.last().copied().unwrap_or("");
    let Some(v) = imm.strip_prefix('#') else { return Err(format!("unrecognised stack adjustment: {}", line.trim())) };
    let parsed = match v.strip_prefix("0x") {
        Some(h) => u64::from_str_radix(h, 16),
        None => v.parse(),
    };
    parsed.map_err(|_| format!("unparsed immediate: {}", line.trim()))
}

#[test]
fn stack_budget() {
    let dir = monoprop_dir();
    let target_dir = dir.join("target").join("stack-budget");
    let built = Command::new(env!("CARGO"))
        .current_dir(&dir)
        .args(["build", "-p", "Drone", "--release", "--target", "thumbv7em-none-eabihf"])
        .env("CARGO_TARGET_DIR", &target_dir)
        .status();
    assert!(built.is_ok_and(|s| s.success()), "release firmware build failed");
    let elf = target_dir.join("thumbv7em-none-eabihf").join("release").join("Drone");
    let tool = objdump();
    assert!(tool.is_some(), "no disassembler for ARM ELF found (llvm-objdump, rust-objdump or /usr/bin/objdump)");
    let Some(tool) = tool else { return };

    // configured stack: the ELF's __stack_size, and the value in .cargo/config.toml
    let syms = Command::new(&tool).arg("-t").arg(&elf).output();
    let syms = syms.map(|o| String::from_utf8_lossy(&o.stdout).into_owned()).unwrap_or_default();
    let stack = syms
        .lines()
        .find(|l| l.trim_end().ends_with(" __stack_size"))
        .and_then(|l| u64::from_str_radix(l.split_whitespace().next().unwrap_or(""), 16).ok());
    assert!(stack.is_some(), "__stack_size not found in the firmware");
    let Some(stack) = stack else { return };
    let config = std::fs::read_to_string(dir.join(".cargo").join("config.toml")).unwrap_or_default();
    let configured = config
        .lines()
        .find_map(|l| l.trim().strip_prefix("TEENSY4_STACK_SIZE"))
        .and_then(|v| v.trim_start_matches([' ', '=']).trim().trim_matches('"').strip_suffix('k')?.parse::<u64>().ok())
        .map(|k| k * 1024);
    assert_eq!(configured, Some(stack), "the firmware's stack differs from TEENSY4_STACK_SIZE in .cargo/config.toml");

    // stack frame of every function
    let dis = Command::new(&tool).args(["-d", "--no-show-raw-insn", "--demangle"]).arg(&elf).output();
    let dis = dis.map(|o| String::from_utf8_lossy(&o.stdout).into_owned()).unwrap_or_default();
    let mut frames: HashMap<String, u64> = HashMap::new();
    let mut current: Option<String> = None;
    let mut errors = Vec::new();
    for line in dis.lines() {
        let t = line.trim();
        if t.ends_with(">:") {
            current = t.split_once(" <").map(|(_, n)| n.trim_end_matches(">:").to_string());
            continue;
        }
        let Some(f) = current.as_ref() else { continue };
        let insn = t.split_once(':').map(|(_, i)| i.trim()).unwrap_or("");
        let bytes = if insn.starts_with("push") || insn.starts_with("vpush") {
            push_bytes(insn)
        } else if insn.starts_with("sub") {
            match sub_sp_bytes(insn) {
                Ok(b) => b,
                Err(e) => {
                    errors.push(format!("{}: {}", f, e));
                    0
                }
            }
        } else {
            0
        };
        *frames.entry(f.clone()).or_default() += bytes;
    }
    assert!(errors.is_empty(), "stack adjustments the test can't size: {:?}", errors);
    let find = |pat: &str| frames.iter().filter(|(n, _)| n.contains(pat)).map(|(_, b)| *b).max().unwrap_or(0);
    let main = find("__cortex_m_rt_main");
    let step = find("Navigator>::step");
    let update = find("Esekf>::update::<3>");
    let helper = frames
        .iter()
        .filter(|(n, _)| !n.contains("__cortex_m_rt_main") && !n.contains("Navigator>::step") && !n.contains("Esekf>::update::<3>"))
        .map(|(_, b)| *b)
        .max()
        .unwrap_or(0);
    let total = main + step + update + helper;
    println!(
        "stack: main {} + Navigator::step {} + Esekf::update::<3> {} + largest other frame {} = {} bytes; stack {} bytes (half = {})",
        main, step, update, helper, total, stack, stack / 2
    );
    assert!(step > 0 && update > 0, "the Navigator's step and update frames were not found");
    assert!(total <= stack / 2, "worst stack path {} bytes exceeds half the {} byte stack", total, stack);
}
