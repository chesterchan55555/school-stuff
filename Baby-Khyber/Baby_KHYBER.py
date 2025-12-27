# ## Demonstration of a Crystals KHYBER PKE with baby parameters
## implement a good baby khyber and a bad one, and show brute-force and lattice attacks
import itertools
import random
import math

# PARAMETERS
n = 8           
k = 2           
q = 97         
secret_space = [-1, 0, 1]   # secret coefficients
error_space  = [-1, 0, 1]   # error coefficients

random.seed(0)

# ---- ring arithmetic: negacyclic mod x^n + 1, coefficients mod q ----
def poly_zero():
    return [0]*n

def poly_add(a,b):
    return [ (a[i] + b[i]) % q for i in range(n) ]

def poly_sub(a,b):
    return [ (a[i] - b[i]) % q for i in range(n) ]

def poly_mul(a,b):
    # negacyclic convolution: x^n = -1
    res = [0]*n
    for i in range(n):
        for j in range(n):
            idx = (i + j) % n
            sign = 1 if (i + j) < n else -1
            res[idx] = (res[idx] + sign * a[i] * b[j]) % q
    return res

def poly_sample(space):
    return [ random.choice(space) for _ in range(n) ]

# ---- vector-of-polynomials functions (k-length) ----
def vec_A_mul_s(A, s):
    # A: k x k list of polys; s: list length k of polys
    out = [poly_zero() for _ in range(k)]
    for i in range(k):
        acc = poly_zero()
        for j in range(k):
            acc = poly_add(acc, poly_mul(A[i][j], s[j]))
        out[i] = acc
    return out

def vec_add(u,v):
    return [ poly_add(u[i], v[i]) for i in range(len(u)) ]

# ---- KeyGen / Encrypt / Decrypt (toy PKE) ----
def keygen():
    # generate random A (k x k matrix of polynomials)
    A = [ [ [ random.randrange(q) for _ in range(n) ] for _ in range(k) ] for _ in range(k) ]
    s = [ poly_sample(secret_space) for _ in range(k) ]
    e = [ poly_sample(error_space) for _ in range(k) ]
    t = vec_add(vec_A_mul_s(A, s), e)   # t = A*s + e
    return (A, t), s

def encrypt(m_int, pk):
    # encode tiny integer m_int into polynomial (constant term)
    A, t = pk
    r = [ poly_sample(secret_space) for _ in range(k) ]
    e1 = [ poly_sample(error_space) for _ in range(k) ]
    e2 = poly_sample(error_space)
    u = vec_add(vec_A_mul_s(A, r), e1)  # u = A r + e1
    dot = poly_zero()
    for i in range(k):
        dot = poly_add(dot, poly_mul(t[i], r[i]))
    mpoly = [ m_int % q ] + [0]*(n-1)
    v = poly_add(poly_add(dot, e2), mpoly)  # v = t^T r + e2 + encode(m)
    return (u, v)

def decrypt(ct, sk):
    u, v = ct
    s = sk
    dot = poly_zero()
    for i in range(k):
        dot = poly_add(dot, poly_mul(s[i], u[i]))
    mm = poly_sub(v, dot)
    # decode constant term (toy scheme)
    return mm[0] % q

# ---- Brute-force attack: enumerate all short secrets and test delta smallness ----
def is_small_delta(delta_poly, tol):
    # delta_poly coefficients -> symmetric representative in [-q//2, q//2]
    for c in delta_poly:
        c_int = c
        if c_int > q//2:
            c_int -= q
        if abs(c_int) > tol:
            return False
    return True

def brute_force_secret(pk, tol=5, max_found=10):
    A, t = pk
    found = []
    total = len(secret_space)**(k*n)
    print("Brute-force space size:", total)
    for flat in itertools.product(secret_space, repeat=k*n):
        # pack into k polys
        s_cand = []
        it = iter(flat)
        for i in range(k):
            s_cand.append([ next(it) for _ in range(n) ])
        As = vec_A_mul_s(A, s_cand)
        delta = [ poly_sub(t[i], As[i]) for i in range(k) ]
        ok = all(is_small_delta(d, tol) for d in delta)
        if ok:
            found.append((s_cand, delta))
            if len(found) >= max_found:
                break
    return found

# ---- Demo run ----
if __name__ == "__main__":
    print("=== Toy PKE demo ===")
    pk, sk = keygen()
    print("Public A (toy):", pk[0])
    print("Public t (toy):", pk[1])
    print("Secret s (hidden):", sk)
    m = 2  # tiny message
    ct = encrypt(m, pk)
    print("Ciphertext (u,v):", ct)
    recovered = decrypt(ct, sk)
    print("Recovered message:", recovered)
    assert recovered == m, "Decoding failed for toy run"

    print("\n=== Running brute-force secret search ===")
    candidates = brute_force_secret(pk, tol=5, max_found=5)
    print("Candidates found (up to 5):")
    for s_cand, delta in candidates:
        print("candidate s:", s_cand, "delta:", delta)

    print("\nDone. Increase n/k to make brute-force infeasible.")
    
