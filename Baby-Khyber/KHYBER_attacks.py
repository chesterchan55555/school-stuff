# import itertools
# import random
# import math
 
# from fpylll import IntegerMatrix, BKZ, BKZParam, LLL
# n = 4      # poly degree
# k = 1      # module rank (1 = ring-style)
# q = 97     # modulus (small)
# secret_space = [-1,0,1]
# error_space = [-1,0,1]
# random.seed(0)

# # ---- ring arithmetic (negacyclic) helpers ----
# def poly_zero(): return [0]*n
# def poly_add(a,b): return [ (a[i]+b[i]) % q for i in range(n) ]
# def poly_sub(a,b): return [ (a[i]-b[i]) % q for i in range(n) ]
# def poly_mul(a,b):
#     res = [0]*n
#     for i in range(n):
#         for j in range(n):
#             idx = (i + j) % n
#             sign = 1 if (i + j) < n else -1
#             res[idx] = (res[idx] + sign * a[i] * b[j]) % q
#     return res

# def sample_small(space):
#     return [ random.choice(space) for _ in range(n) ]

# # ---- Keygen (toy) returning A (k x k) and t (k vector) and secret s ----
# def keygen():
#     A = [ [ [ random.randrange(q) for _ in range(n) ] for _ in range(k) ] for _ in range(k) ]
#     s = [ sample_small(secret_space) for _ in range(k) ]
#     e = [ sample_small(error_space) for _ in range(k) ]
#     # compute t = A*s + e
#     t = []
#     for i in range(k):
#         acc = poly_zero()
#         for j in range(k):
#             acc = poly_add(acc, poly_mul(A[i][j], s[j]))
#         t.append(poly_add(acc, e[i]))
#     return (A, t), s

# # ---- Build negacyclic circulant matrix C(a) (n x n) for polynomial a ----
# def C_of_a(a):
#     # a: length-n list
#     M = [ [0]*n for _ in range(n) ]
#     for r in range(n):
#         for c in range(n):
#             idx = r - c
#             if idx < 0:
#                 idx += n
#                 M[r][c] = (-a[idx]) % q
#             else:
#                 M[r][c] = a[idx] % q
#     return M

# # ---- Build q-ary basis B (kn x kn) standard construction ----
# def build_qary_basis(A):
#     kn = k * n
#     B = [ [0]* (kn) for _ in range(kn) ]
#     # Top-left block: q * I_n
#     for i in range(n):
#         B[i][i] = q
#     # For rows block r = 1..k: place -C(a_r) in cols 0..n-1 and I on diagonal block
#     # We'll follow the common construction:
#     # Row-block 0: q I_n | 0 | 0 ...
#     # Row-block i (1..k): -C(a_i) | I_n at block (i)
#     for block in range(1, k+1):
#         row_start = block * n
#         # a_index: block-1
#         a_poly = A[block-1][0]  # since k=1 this is okay; generalize if k>1
#         C = C_of_a(a_poly)
#         # place -C in columns 0..n-1 (leftmost block)
#         for i in range(n):
#             for j in range(n):
#                 B[row_start + i][j] = (-C[i][j]) % q
#         # place identity on the diagonal block (columns row_start..row_start+n-1)
#         if block < k+1:
#             for i in range(n):
#                 B[row_start + i][row_start + i] = 1
#     # Note: This construction is a small variant used for demonstration.
#     return B

# # ---- Helper: convert Python matrix to fpylll IntegerMatrix and run BKZ/LLL ----
# def matrix_to_fpylll(M):
#     rows = len(M); cols = len(M[0])
#     mat = IntegerMatrix(rows, cols)
#     for i in range(rows):
#         for j in range(cols):
#             mat[i,j] = int(M[i][j])
#     return mat

# # ---- Demo: run BKZ and inspect short vectors ----
# if __name__ == "__main__":
#     (A, t), s = keygen()
#     print("Public A:", A)
#     print("Public t:", t)
#     print("Secret s (hidden):", s)

#     B = build_qary_basis(A)
#     print("\nq-ary basis B ({}x{}):".format(len(B), len(B[0])))
#     for row in B:
#         print(row)

#     M = matrix_to_fpylll(B)
#     print("\nRunning LLL (fast) ...")
#     LLL.reduction(M)
#     for i in range(min(8, M.nrows)):
#         row = [int(M[i,j]) for j in range(M.ncols)]
#         norm2 = sum(x*x for x in row)
#         print("LLL row", i, "norm^2", norm2, "row:", row)

#     # Now try BKZ with a small block size (beta)
#     beta = 20  # small for toy; increase to see stronger reduction (and heavier compute)
#     print("\nRunning BKZ with beta =", beta, " (may take a bit) ...")
#     params = BKZParam(block_size=beta)
#     BKZ.reduction(M, params)
#     print("After BKZ (first few rows):")
#     for i in range(min(8, M.nrows)):
#         row = [int(M[i,j]) for j in range(M.ncols)]
#         norm2 = sum(x*x for x in row)
#         print("BKZ row", i, "norm^2", norm2, "row:", row)

#     # Post-processing: look for short vectors with small coefficients (candidate secrets)
#     threshold = 1000
#     print("\nLooking for short rows with small coefficients (threshold norm^2 <=", threshold, ")")
#     candidates = []
#     for i in range(M.nrows):
#         row = [int(M[i,j]) for j in range(M.ncols)]
#         norm2 = sum(x*x for x in row)
#         if norm2 <= threshold:
#             # interpret row as block [z1 | z2 | ...] of length k*n
#             # for k=1 we expect z (candidate) in cols 0..n-1 maybe with q-multiples etc.
#             candidates.append((i, norm2, row))
#     for idx, norm2, row in candidates:
#         print("candidate row", idx, "norm2", norm2, "row:", row)

#     print("\nDone. For larger n/k you need larger BKZ beta and more resources.")