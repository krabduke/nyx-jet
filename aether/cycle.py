"""
Aether AX-1 -- the thermodynamic cycle, at sea-level static, ISA.

Pure Python, no bpy. `spec.py` takes the engine's headline figures from here
rather than typing them in, so the thrust, the bypass ratio and the nozzle
throat area the geometry is drawn to all come out of one calculation and
cannot drift apart. `verify.py` then reads the built flowpath back and checks
the axial Mach number it implies at every station against this cycle.

WHAT IS MODELLED
----------------
A mixed-flow afterburning turbofan with a third stream:

    0  ambient              13  fan exit, all flow
    2  engine face          3S  third stream, split off at the fan exit tip
    21 CDFS exit            16  second (bypass) stream, split after the CDFS
    25 HPC inlet (core)     3   HPC exit
    4  combustor exit       45  HPT exit / LPT inlet (cooling air re-mixed)
    5  LPT exit             6   mixed-out core + bypass
    7  augmentor exit       8   2D nozzle throat,  9  exit

The core-driven fan stage (CDFS) sits on the HP spool and works on both the
core and the bypass air -- that is what lets the engine move its bypass ratio
without changing fan speed, and it is why the HP turbine has more to do than
the compressor alone would ask of it.

The bypass ratio is NOT an input. A mixed-flow turbofan only works if the
core and the bypass reach the mixer at about the same total pressure --
otherwise one stream backs up into the other -- and for a given fan, core and
turbine inlet temperature there is exactly one bypass ratio that does that.
`design()` solves for it by bisection.

WHAT IS NOT
-----------
Constant specific heats in two gases (cold air, hot products) rather than
temperature-dependent properties; polytropic component efficiencies rather
than maps; cooling air lumped as one chargeable bleed re-mixed ahead of the LP
turbine; ideal full expansion in the nozzles with a velocity coefficient. That
is textbook preliminary-design fidelity (Saravanamuttoo, "Gas Turbine
Theory", ch. 3; Mattingly, "Elements of Propulsion", ch. 8) -- good to a few
per cent on thrust and SFC, and honest about being no better.
"""

import math

# --------------------------------------------------------------------------
# Gas properties
# --------------------------------------------------------------------------

R_GAS = 287.05
CP_AIR, G_AIR = 1004.5, 1.400
CP_HOT, G_HOT = 1148.0, 1.333
LHV = 43.1e6                  # J/kg, JP-8

T_AMB, P_AMB = 288.15, 101325.0


def _tau(pr, gamma, eta_poly, compress=True):
    """Total-temperature ratio across a compressor (or turbine) of pressure
    ratio pr at polytropic efficiency eta_poly."""
    n = (gamma - 1.0) / gamma
    return pr ** (n / eta_poly) if compress else pr ** (-n * eta_poly)


def _pr_turbine(tau, gamma, eta_poly):
    """Pressure ratio (in / out, > 1) of a turbine with temperature ratio tau."""
    n = (gamma - 1.0) / gamma
    return tau ** (-1.0 / (n * eta_poly))


def _jet(tt, pt, cp, gamma, cv):
    """Fully expanded jet velocity from total conditions."""
    pr = P_AMB / pt
    if pr >= 1.0:
        return 0.0
    return cv * math.sqrt(2.0 * cp * tt * (1.0 - pr ** ((gamma - 1.0) / gamma)))


def throat_area(mdot, tt, pt, gamma, cp):
    """Choked throat area, m^2, for a stream of total conditions tt, pt."""
    r = cp * (gamma - 1.0) / gamma
    flow_fn = math.sqrt(gamma / r) * (2.0 / (gamma + 1.0)) ** (
        (gamma + 1.0) / (2.0 * (gamma - 1.0)))
    return mdot * math.sqrt(tt) / (pt * flow_fn)


def axial_mach(mdot, tt, pt, area, gamma=G_AIR, cp=CP_AIR):
    """Subsonic Mach number that passes mdot through `area` (m^2) at the given
    total conditions -- the inverse of the mass-flow function, by bisection.
    Returns None if the area is too small to pass the flow even choked."""
    r = cp * (gamma - 1.0) / gamma

    def flow(m):
        t = 1.0 + 0.5 * (gamma - 1.0) * m * m
        return (math.sqrt(gamma / r) * m * t ** (-(gamma + 1.0) / (2.0 * (gamma - 1.0)))
                * pt * area / math.sqrt(tt))

    if flow(1.0) < mdot:
        return None
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if flow(mid) < mdot:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# --------------------------------------------------------------------------
# Design point
# --------------------------------------------------------------------------

DESIGN = {
    # Total airflow at the engine face. Sized so the headline thrust lands on
    # what a twin-engine agile fighter of ~19 t combat weight needs for a
    # thrust-to-weight of about 1.4 on reheat; see the Nyx repo for the other
    # half of that sum.
    "w_total":      112.0,    # kg/s
    # Two transonic fan stages. 3.8 over two stages is 1.95 a stage, which is
    # the top end of what a blisked, forward-swept stage does today.
    "fpr":            3.8,
    # One core-driven fan stage on the HP spool, working on core + bypass.
    "cdfs_pr":        1.30,
    # Six HP compressor stages at an average of 1.40 a stage.
    "hpc_pr":         7.40,
    # Combustor exit. 2,050 K is ceramic-matrix-composite territory: a nickel
    # liner could not live here, which is why the liners are CMC.
    "t4":          2050.0,    # K
    # Third stream at the maximum-thrust setting of the mode valve: most of
    # the fan air is sent through the core and bypass, and the third stream
    # carries only what it needs to cool the nozzle and sink the aircraft's
    # heat load. The valve opens it to ~0.24 at cruise.
    "beta3":          0.10,
    # Heat the aircraft rejects into the third stream (avionics, sensors,
    # directed-energy load), and shaft power taken off the HP spool for the
    # generators. An adaptive engine is sold as much on these two numbers as
    # on thrust.
    "q_tms":        350e3,    # W into the third stream
    "p_offtake":    400e3,    # W off the HP spool
    # Chargeable cooling air, as a fraction of core flow, taken at HPC exit
    # and returned ahead of the LP turbine.
    "eps_cool":       0.12,
    # Augmentor exit temperature, lit.
    "t7_ab":       2150.0,    # K

    # polytropic efficiencies
    "e_fan": 0.89, "e_cdfs": 0.89, "e_hpc": 0.905,
    "e_hpt": 0.885, "e_lpt": 0.905,
    "eta_b": 0.998, "eta_ab": 0.94, "eta_mech": 0.99,
    # total-pressure recoveries
    "pi_inlet": 0.985, "pi_duct3": 0.97, "pi_duct2": 0.975,
    "pi_burner": 0.955, "pi_mixer": 0.98,
    "pi_ab_dry": 0.97, "pi_ab_wet": 0.93,
    # nozzle velocity coefficients
    "cv_main": 0.985, "cv_third": 0.975,
    # the mixer is balanced at this total-pressure ratio, core over bypass
    "mixer_pr_target": 1.00,
}


def _run(d, bpr2):
    """One pass of the cycle at a trial second-stream bypass ratio.

    bpr2 is second-stream flow over core flow. Returns a dict of everything.
    """
    s = {}
    w = d["w_total"]
    w3 = d["beta3"] * w
    w_in = w - w3                          # through the CDFS
    w_core = w_in / (1.0 + bpr2)
    w2 = w_in - w_core                     # second stream

    t2 = T_AMB
    p2 = P_AMB * d["pi_inlet"]

    # fan
    tau_f = _tau(d["fpr"], G_AIR, d["e_fan"])
    t13, p13 = t2 * tau_f, p2 * d["fpr"]
    work_fan = w * CP_AIR * (t13 - t2)

    # third stream: ducted round the outside, heated by the aircraft's heat load
    t3s = t13 + d["q_tms"] / (w3 * CP_AIR)
    p3s = p13 * d["pi_duct3"]

    # CDFS on the HP spool, working on core + bypass
    tau_c = _tau(d["cdfs_pr"], G_AIR, d["e_cdfs"])
    t21, p21 = t13 * tau_c, p13 * d["cdfs_pr"]
    work_cdfs = w_in * CP_AIR * (t21 - t13)

    # second stream
    t16, p16 = t21, p21 * d["pi_duct2"]

    # HP compressor
    tau_h = _tau(d["hpc_pr"], G_AIR, d["e_hpc"])
    t3, p3 = t21 * tau_h, p21 * d["hpc_pr"]
    work_hpc = w_core * CP_AIR * (t3 - t21)

    # combustor, less the cooling air
    w_cool = d["eps_cool"] * w_core
    w_burn = w_core - w_cool
    t4, p4 = d["t4"], p3 * d["pi_burner"]
    # fuel from an energy balance: air at t3 plus fuel -> products at t4
    f = (CP_HOT * t4 - CP_AIR * t3) / (d["eta_b"] * LHV - CP_HOT * t4)
    wf = f * w_burn
    w4 = w_burn + wf

    # HP turbine: compressor, CDFS and the generators
    work_hp = (work_hpc + work_cdfs + d["p_offtake"]) / d["eta_mech"]
    t41 = t4 - work_hp / (w4 * CP_HOT)
    tau_ht = t41 / t4
    p41 = p4 / _pr_turbine(tau_ht, G_HOT, d["e_hpt"])

    # cooling air re-mixed ahead of the LP turbine
    w45 = w4 + w_cool
    t45 = (w4 * CP_HOT * t41 + w_cool * CP_AIR * t3) / (w45 * CP_HOT)
    p45 = p41

    # LP turbine drives the fan
    work_lp = work_fan / d["eta_mech"]
    t5 = t45 - work_lp / (w45 * CP_HOT)
    tau_lt = t5 / t45
    p5 = p45 / _pr_turbine(tau_lt, G_HOT, d["e_lpt"])

    # mixer: enthalpy-conserving, mass-weighted properties
    w6 = w45 + w2
    cp6 = (w45 * CP_HOT + w2 * CP_AIR) / w6
    t6 = (w45 * CP_HOT * t5 + w2 * CP_AIR * t16) / (w6 * cp6)
    # mixed-out pressure: flow-weighted, less the mixer loss
    p6 = (w45 * p5 + w2 * p16) / w6 * d["pi_mixer"]
    g6 = cp6 / (cp6 - R_GAS)

    s.update(dict(w=w, w3=w3, w2=w2, w_core=w_core, w_cool=w_cool,
                  w4=w4, w45=w45, w6=w6, wf=wf, far=f, bpr2=bpr2,
                  bpr_total=(w2 + w3) / w_core,
                  t2=t2, p2=p2, t13=t13, p13=p13, t3s=t3s, p3s=p3s,
                  t21=t21, p21=p21, t16=t16, p16=p16, t3=t3, p3=p3,
                  t4=t4, p4=p4, t41=t41, p41=p41, t45=t45, p45=p45,
                  t5=t5, p5=p5, t6=t6, p6=p6, cp6=cp6, g6=g6,
                  work_fan=work_fan, work_cdfs=work_cdfs, work_hpc=work_hpc,
                  hpt_pr=p4 / p41, lpt_pr=p45 / p5,
                  opr=p3 / p2, mixer_pr=p5 / p16))
    return s


def design(d=None):
    """Solve the cycle at the design point. Returns a dict of stations and
    performance, dry and augmented."""
    d = dict(DESIGN, **(d or {}))

    # bisection on the bypass ratio for a balanced mixer. More bypass means
    # less core flow for the same fan work, so the LP turbine takes a bigger
    # temperature drop and the core arrives at the mixer at a lower pressure:
    # mixer_pr falls monotonically as bpr2 rises.
    lo, hi = 0.02, 3.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if _run(d, mid)["mixer_pr"] > d["mixer_pr_target"]:
            lo = mid
        else:
            hi = mid
    s = _run(d, 0.5 * (lo + hi))

    # --- dry ---
    p7 = s["p6"] * d["pi_ab_dry"]
    v9 = _jet(s["t6"], p7, s["cp6"], s["g6"], d["cv_main"])
    v3s = _jet(s["t3s"], s["p3s"], CP_AIR, G_AIR, d["cv_third"])
    f_main = s["w6"] * v9
    f_third = s["w3"] * v3s
    thrust_dry = f_main + f_third
    a8_dry = throat_area(s["w6"], s["t6"], p7, s["g6"], s["cp6"])

    # --- augmented ---
    t7 = d["t7_ab"]
    cp7, g7 = CP_HOT, G_HOT
    f_ab = (cp7 * t7 - s["cp6"] * s["t6"]) / (d["eta_ab"] * LHV - cp7 * t7)
    wf_ab = f_ab * s["w6"]
    w7 = s["w6"] + wf_ab
    p7w = s["p6"] * d["pi_ab_wet"]
    v9w = _jet(t7, p7w, cp7, g7, d["cv_main"])
    thrust_wet = w7 * v9w + f_third
    a8_wet = throat_area(w7, t7, p7w, g7, cp7)

    s.update(dict(
        p7_dry=p7, v9_dry=v9, v3s=v3s, thrust_dry=thrust_dry,
        thrust_third=f_third, a8_dry=a8_dry,
        t7=t7, p7_wet=p7w, w7=w7, wf_ab=wf_ab, v9_wet=v9w,
        thrust_wet=thrust_wet, a8_wet=a8_wet,
        sfc_dry=s["wf"] / thrust_dry * 1e6,                 # mg/(N s)
        sfc_wet=(s["wf"] + wf_ab) / thrust_wet * 1e6,
        spec_thrust_dry=thrust_dry / s["w"],
        spec_thrust_wet=thrust_wet / s["w"],
        # nozzle pressure ratio, which sets how much divergence the nozzle needs
        npr_dry=p7 / P_AMB, npr_wet=p7w / P_AMB,
    ))
    # area ratio A9/A8 for full expansion at the augmented NPR
    s["a9_a8_wet"] = _area_ratio(s["npr_wet"], g7)
    s["a9_a8_dry"] = _area_ratio(s["npr_dry"], s["g6"])
    s["inputs"] = d
    return s


def _area_ratio(npr, g):
    """Exit-to-throat area ratio for full expansion at pressure ratio npr."""
    m9 = math.sqrt(2.0 / (g - 1.0) * (npr ** ((g - 1.0) / g) - 1.0))
    return (1.0 / m9) * ((2.0 / (g + 1.0)) * (1.0 + 0.5 * (g - 1.0) * m9 * m9)) ** (
        (g + 1.0) / (2.0 * (g - 1.0)))


def report(s=None):
    s = s or design()
    kn = lambda n: n / 1000.0
    lines = [
        "Aether AX-1 cycle, sea-level static ISA",
        f"  airflow            {s['w']:7.1f} kg/s   core {s['w_core']:.1f}, "
        f"bypass {s['w2']:.1f}, third {s['w3']:.1f}",
        f"  bypass ratio       {s['bpr2']:7.3f}  (second stream, solved for a "
        f"balanced mixer); {s['bpr_total']:.3f} with the third stream",
        f"  pressure ratios    fan {s['p13']/s['p2']:.2f}  CDFS "
        f"{s['p21']/s['p13']:.2f}  HPC {s['p3']/s['p21']:.2f}  overall "
        f"{s['opr']:.1f}",
        f"  turbine            T4 {s['t4']:.0f} K, HPT PR {s['hpt_pr']:.2f}, "
        f"LPT PR {s['lpt_pr']:.2f}, T5 {s['t5']:.0f} K",
        f"  mixer              core/bypass Pt {s['mixer_pr']:.3f}, T6 "
        f"{s['t6']:.0f} K",
        f"  dry thrust         {kn(s['thrust_dry']):7.1f} kN  SFC "
        f"{s['sfc_dry']:.1f} mg/Ns  NPR {s['npr_dry']:.2f}  A8 "
        f"{s['a8_dry']:.3f} m2",
        f"  reheat thrust      {kn(s['thrust_wet']):7.1f} kN  SFC "
        f"{s['sfc_wet']:.1f} mg/Ns  NPR {s['npr_wet']:.2f}  A8 "
        f"{s['a8_wet']:.3f} m2  A9/A8 {s['a9_a8_wet']:.2f}",
        f"  third stream       {kn(s['thrust_third']):7.2f} kN at "
        f"{s['v3s']:.0f} m/s, {s['t3s'] - 273.15:.0f} C",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    print(report())
