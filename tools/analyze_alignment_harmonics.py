"""Explore nominal current harmonics in a bidirectional alignment; never exports drive compensation."""
import argparse
import cmath
import json
import math
from pathlib import Path
import statistics
from analyze_current_balance import load_vectors
from analyze_ripple import solve_weighted


def fit(angles, values, orders):
    basis = [[1] + [x for order in orders for x in (math.cos(order*a), math.sin(order*a))] for a in angles]
    coefficients = solve_weighted(basis, values, [1]*len(values))
    if coefficients is None:
        raise ValueError("Degenerate alignment samples")
    return coefficients


def predict_shape(angle, coefficients, orders):
    return sum(coefficients[1+2*n]*math.cos(order*angle) + coefficients[2+2*n]*math.sin(order*angle)
               for n, order in enumerate(orders))


def describe(coefficients, orders):
    return [{"order": order, "cos_nominal_A": coefficients[1+2*n], "sin_nominal_A": coefficients[2+2*n],
             "amplitude_nominal_A": math.hypot(coefficients[1+2*n], coefficients[2+2*n])}
            for n, order in enumerate(orders)]


def analyze(rows, vectors):
    if "leg" not in rows[0]:
        raise ValueError("Requires an alignment survey")
    a0 = rows[0]["encoder_angle_rad"]
    last_forward = [r for r in rows if r["leg"] == 0][-1]
    direction = 1 if last_forward["encoder_angle_rad"] > a0 else -1
    samples = []
    for row, (a, b, c) in zip(rows, vectors):
        if row["leg"] not in (0, 1) or not .3 < row["field_displacement_rad"] < 18.6:
            continue
        angle = direction*(row["encoder_angle_rad"]-a0)
        current = complex((2*a-b-c)/3, (b-c)/math.sqrt(3))*cmath.exp(-1j*7*angle)
        samples.append((int(row["leg"]), angle, current))
    # A common frame for both directions, derived from their mean current.
    # This is not an independent determination of the rotor flux angle.
    center = sum(sample[2] for sample in samples)/len(samples)
    frame_angle = cmath.phase(center)
    legs = {}
    for leg in (0, 1):
        selected = [(angle, current*cmath.exp(-1j*frame_angle)) for which, angle, current in samples if which == leg]
        legs[leg] = ([a for a, _ in selected], [v.imag for _, v in selected], [v.real for _, v in selected])
    models = []
    for orders in ([84], [14, 42, 84]):
        fitted = {leg: fit(legs[leg][0], legs[leg][1], orders) for leg in (0, 1)}
        groups = []
        for leg in (0, 1):
            angles, values, direct = legs[leg]
            opposite = fitted[1-leg]
            prediction = [predict_shape(a, opposite, orders) for a in angles]
            # Only the harmonic shape is transferred. Remove DC separately
            # because directional friction has opposite signs in the two legs.
            mean_value, mean_prediction = statistics.mean(values), statistics.mean(prediction)
            errors = [(v-mean_value)-(p-mean_prediction) for v, p in zip(values, prediction)]
            total = sum((v-mean_value)**2 for v in values)
            groups.append({"leg": leg, "samples": len(values), "mean_q_nominal_A": mean_value,
                           "mean_d_nominal_A": statistics.mean(direct),
                           "fitted_harmonics": describe(fitted[leg], orders),
                           "opposite_leg_shape_explained_variance": 1-sum(e*e for e in errors)/total if total else None,
                           "opposite_leg_shape_error_rms_nominal_A": math.sqrt(statistics.mean(e*e for e in errors))})
        models.append({"orders": orders, "legs": groups})
    return {"common_frame_angle_rad": frame_angle, "models": models,
            "note": "Exploratory nominal current in a frame derived from the bidirectional mean; not calibrated rotor torque. Opposite-leg shape evaluation removes each leg's DC and is not absolute prediction. Sensor nonlinearity, rotor dynamics, ADC/drive distortion and friction remain confounded. No firmware correction is exported."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rows, vectors = load_vectors(args.log)
    output = json.dumps({"log": args.log.name, **analyze(rows, vectors)}, indent=2) + "\n"
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
