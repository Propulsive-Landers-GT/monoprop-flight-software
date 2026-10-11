// Minimal test harness: named tests and `assert_close`, counting failures.
use std::sync::atomic::{AtomicUsize, Ordering};

static PASSED: AtomicUsize = AtomicUsize::new(0);
static FAILED: AtomicUsize = AtomicUsize::new(0);

/// Run one named test. A panic (assert!, assert_eq!, unwrap on None, ...) marks it failed.
pub fn test<F: FnOnce()>(name: &str, f: F) {
    std::panic::set_hook(Box::new(|_| {}));
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
    let _ = std::panic::take_hook();
    match r {
        Ok(()) => {
            PASSED.fetch_add(1, Ordering::SeqCst);
            println!("  ok      {}", name);
        }
        Err(e) => {
            FAILED.fetch_add(1, Ordering::SeqCst);
            let msg = if let Some(s) = e.downcast_ref::<&str>() {
                s.to_string()
            } else if let Some(s) = e.downcast_ref::<String>() {
                s.clone()
            } else {
                "panicked".to_string()
            };
            println!("  FAILED  {}\n          {}", name, msg.replace('\n', "\n          "));
        }
    }
}

/// Panic with a helpful message unless |got - want| <= tol.
#[track_caller]
pub fn assert_close(got: f64, want: f64, tol: f64, what: &str) {
    if !((got - want).abs() <= tol) {
        panic!("{}: got {}, expected {} (tolerance {})", what, got, want, tol);
    }
}

pub fn section(name: &str) {
    println!("\n== {}", name);
}

/// Print the summary and exit non-zero if anything failed (so `cargo test` and CI report it).
pub fn finish() {
    let (p, f) = (PASSED.load(Ordering::SeqCst), FAILED.load(Ordering::SeqCst));
    println!("\n{} passed, {} failed", p, f);
    if f > 0 {
        std::process::exit(1);
    }
}
