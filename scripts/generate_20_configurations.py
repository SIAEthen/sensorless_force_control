import numpy as np

# Joint ranges (degrees):
#   q1: -60 ~ 60
#   q2: -35 ~ 35
#   q3: -35 ~ 35
#   q4:  0  (fixed)
#   q5:  0  ~ 180
#   q6:  0  (fixed)

def generate_configurations(n: int, seed: int = 10):
    rng = np.random.default_rng(seed)
    configs = []
    for _ in range(n):
        q1 = rng.uniform(-90, 90)
        q2 = rng.uniform(-35, 35)
        q3 = rng.uniform(-35, 35)
        q4 = rng.uniform(-45, 45)
        q5 = q2 + q3 - 90 + rng.uniform(-25, 25)
        q6 = rng.uniform(0, 120)
        configs.append([q1, q2, q3, q4, q5, q6])
    return configs


if __name__ == "__main__":
    N = 50
    configs = generate_configurations(N)
    print(f"{'#':>3}  {'q1':>8}  {'q2':>8}  {'q3':>8}  {'q4':>8}  {'q5':>8}  {'q6':>8}  (deg)")
    print("-" * 70)
    for i, q in enumerate(configs):
        print(f"{i+1:>3}  {q[0]:>8.2f}  {q[1]:>8.2f}  {q[2]:>8.2f}  {q[3]:>8.2f}  {q[4]:>8.2f}  {q[5]:>8.2f}")
