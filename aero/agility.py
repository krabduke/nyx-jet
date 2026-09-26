"""How hard Nyx can turn, and where it runs out: energy manoeuvrability.

    python3 aero/agility.py

The standard method (Boyd and Christie's energy-manoeuvrability theory):
turn rate against speed, bounded by three limits --

  lift       the most lift the wing can make at that dynamic pressure
  structure  the load factor the airframe is built for (9.5 g)
  thrust     the load factor at which drag equals the available thrust,
             i.e. the fastest turn that can be held without losing energy

The intersection of the first two is the corner speed, where the
instantaneous turn rate peaks; the peak of the third is the best sustained
turn rate.

The lift limit is not an attached-flow stall. A delta of aspect ratio 2.6
flies on its leading-edge vortex well past where attached flow would give
up, and the right model is Polhamus's leading-edge-suction analogy
(NASA TN D-3767, 1966):

    CL = Kp sin a cos^2 a + Kv cos a sin^2 a

with Kp the potential-flow lift slope -- taken from the vortex-lattice solve
of this aircraft, canards included, so it is this aircraft's and not a
formula's -- and Kv = pi, the slender-delta limiting value. The angle of
attack is capped at 28 degrees for manoeuvring, below vortex breakdown on a
48-degree delta with close-coupled canards; that cap is an assumption.

The drag polar is CD = CD0 + K CL^2, both measured: an OpenFOAM run of
the built aircraft (the aero study, model-gallery/aero) gave CD 0.0222 at
zero lift and 0.0654 at CL 0.442, so CD0 0.0222 and K 0.222 -- an Oswald
factor of 0.564. They replace the assumptions they overturned, 0.018 and
0.72: the drag was 23 % higher at zero lift than assumed and 28 % higher
due to lift. The CD0 includes the base drag of the powered-off model's
capped nozzles, so it errs high; the band, 0.020-0.024, covers that.

Thrust falls with density as (rho/rho0)^0.7 -- the usual first-order rule,
not an engine deck; there is no Mach-number effect in it, which is why the
study stops at Mach 0.9.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "nyx"))
sys.path.insert(0, os.path.join(HERE, "aero"))
import spec   # noqa: E402

G = 9.80665
RHO0 = 1.225
CD0 = 0.0222
CD0_BAND = (0.020, 0.024)
OSWALD = 0.564
ALPHA_MAX_DEG = 28.0
KV = math.pi
MACH_MAX = 0.9


def atmosphere(h):
    """ISA troposphere: (density, speed of sound)."""
    t = 288.15 - 0.0065 * h
    return RHO0 * (t / 288.15) ** 4.2561, math.sqrt(1.4 * 287.05 * t)


def engine_thrust(afterburner=True):
    """Sea-level static thrust of one engine, from the vendored engine."""
    from parts import engines
    e = engines.info()
    return e["thrust_ab"] if afterburner else e["thrust_dry"]


class Jet:
    def __init__(self, fuel_fraction=spec.COMBAT_FUEL_FRACTION, cd0=CD0):
        import vlm
        self.m = spec.mass(fuel_fraction)
        self.W = self.m * G
        self.S = spec.s_ref_m2()
        b = 2 * spec.WING["y_tip"] / 1000.0
        self.AR = b * b / self.S
        _, self.Kp = vlm.neutral_point()
        self.cd0 = cd0
        self.K = 1.0 / (math.pi * OSWALD * self.AR)
        self.T0 = 2 * engine_thrust(True)

    def cl(self, a):
        s, c = math.sin(a), math.cos(a)
        return self.Kp * s * c * c + KV * c * s * s

    def cl_max(self):
        return self.cl(math.radians(ALPHA_MAX_DEG))

    def thrust(self, h):
        rho, _ = atmosphere(h)
        return self.T0 * (rho / RHO0) ** 0.7

    def n_lift(self, v, h):
        rho, _ = atmosphere(h)
        return 0.5 * rho * v * v * self.S * self.cl_max() / self.W

    def n_available(self, v, h):
        return min(self.n_lift(v, h), spec.G_LIMIT)

    def n_sustained(self, v, h):
        """Load factor at which drag = thrust at speed v."""
        rho, _ = atmosphere(h)
        q = 0.5 * rho * v * v
        T = self.thrust(h)
        x = (T - q * self.S * self.cd0) * q * self.S / (self.K * self.W * self.W)
        return math.sqrt(x) if x > 0 else 0.0

    def turn_rate(self, v, n):
        return math.degrees(G * math.sqrt(max(n * n - 1.0, 0.0)) / v)

    def corner_speed(self, h):
        rho, _ = atmosphere(h)
        return math.sqrt(2 * spec.G_LIMIT * self.W / (rho * self.S * self.cl_max()))

    def ps(self, v, h, n=1.0):
        """Specific excess power at speed v, load factor n: (T - D) V / W."""
        rho, _ = atmosphere(h)
        q = 0.5 * rho * v * v
        cl = n * self.W / (q * self.S)
        D = q * self.S * (self.cd0 + self.K * cl * cl)
        return (self.thrust(h) - D) * v / self.W

    def best_sustained(self, h):
        _, a = atmosphere(h)
        best = (0.0, 0.0, 0.0)
        for i in range(1, 200):
            v = 60.0 + (MACH_MAX * a - 60.0) * i / 199.0
            n = min(self.n_sustained(v, h), self.n_available(v, h))
            r = self.turn_rate(v, n)
            if r > best[0]:
                best = (r, v, n)
        return best

    def instantaneous(self, h):
        v = self.corner_speed(h)
        return self.turn_rate(v, spec.G_LIMIT), v


def report():
    lines = []
    for cd0 in (CD0,) + CD0_BAND:
        j = Jet(cd0=cd0)
        tag = "" if cd0 == CD0 else f"   (CD0 {cd0})"
        for h, name in ((0.0, "sea level"), (4572.0, "15,000 ft")):
            itr, vc = j.instantaneous(h)
            str_, vs, ns = j.best_sustained(h)
            _, a = atmosphere(h)
            if cd0 == CD0:
                lines.append(f"  {name:9s}  instantaneous {itr:5.1f} deg/s at {vc:.0f} m/s "
                             f"(M {vc / a:.2f}), sustained {str_:5.1f} deg/s at "
                             f"{ns:.1f} g, M {vs / a:.2f}")
            else:
                lines.append(f"  {name:9s}  sustained {str_:5.1f} deg/s{tag}")
    j = Jet()
    head = [f"Nyx, combat mass {j.m:,.0f} kg, S {j.S:.1f} m^2, AR {j.AR:.2f}",
            f"  wing loading {j.m / j.S:.0f} kg/m^2, thrust/weight {j.T0 / j.W:.2f} "
            f"(max reheat, sea level)",
            f"  lift slope {j.Kp:.2f} /rad (VLM), CLmax {j.cl_max():.2f} at "
            f"{ALPHA_MAX_DEG:.0f} deg (Polhamus), K {j.K:.3f}"]
    return "\n".join(head + lines)


if __name__ == "__main__":
    print(report())
