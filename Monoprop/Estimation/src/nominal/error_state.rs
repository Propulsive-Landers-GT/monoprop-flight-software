use crate::*;

/// x (+) dx: apply a 15-number error to a nominal state (attitude: q * Exp(dtheta)).
pub fn inject(x: &Nominal, dx: &Vector<15>) -> Nominal {
    let mut q = x.q * quat_exp(&sub3(dx, 6));
    q.renormalize();
    Nominal {
        p: x.p + sub3(dx, 0),
        v: x.v + sub3(dx, 3),
        q,
        ab: x.ab + sub3(dx, 9),
        wb: x.wb + sub3(dx, 12),
    }
}

/// x_t (-) x: the 15-number error that takes x to x_t.
pub fn difference(x_t: &Nominal, x: &Nominal) -> Vector<15> {
    let mut d = Vector::<15>::zeros();
    d.fixed_rows_mut::<3>(0).copy_from(&(x_t.p - x.p));
    d.fixed_rows_mut::<3>(3).copy_from(&(x_t.v - x.v));
    d.fixed_rows_mut::<3>(6).copy_from(&boxminus(&x_t.q, &x.q));
    d.fixed_rows_mut::<3>(9).copy_from(&(x_t.ab - x.ab));
    d.fixed_rows_mut::<3>(12).copy_from(&(x_t.wb - x.wb));
    d
}
