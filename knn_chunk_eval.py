"""Does action-chunk retrieval + temporal ensembling beat single-step kNN on L1?

Reuses the DINOv3 embedding bank (no re-encoding): builds per-frame future N-action chunks
from the bank's actions + episode boundaries, retrieves chunks for each held-out query, and
temporally ensembles the overlapping predictions for each target frame (ACT-style).
Compares L1 vs the single-step baseline.
"""
import argparse
import numpy as np
import torch

ARM = slice(0, 6)
HAND = slice(6, 26)


def build_future(act, epi, N):
    """fut[i,d] = act[i+d] if within the same episode else 0; valid[i,d] = mask."""
    M, A = act.shape
    fut = np.zeros((M, N, A), np.float32)
    valid = np.zeros((M, N), bool)
    for d in range(N):
        idx = np.arange(M) + d
        ok = idx < M
        idxc = np.clip(idx, 0, M - 1)
        same = ok & (epi[idxc] == epi)
        fut[:, d] = np.where(same[:, None], act[idxc], 0.0)
        valid[:, d] = same
    return fut, valid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bank", default="knn_bank_left_vits16.npz")
    ap.add_argument("--holdout_frac", type=float, default=0.1)
    ap.add_argument("--k", type=int, default=32)
    ap.add_argument("--temp", type=float, default=0.05)
    ap.add_argument("--N", type=int, default=16, help="chunk length")
    ap.add_argument("--ens_m", type=float, default=0.1, help="temporal-ensemble decay (weight=exp(-m*d))")
    args = ap.parse_args()

    z = np.load(args.bank)
    emb, act, epi = z["emb"], z["act"], z["epi"]
    eps = np.unique(epi)
    n_test = max(1, int(len(eps) * args.holdout_frac))
    test_eps = set(eps[:n_test].tolist())
    tm = np.isin(epi, list(test_eps))
    bm = ~tm
    keepq = tm & (~np.all(act[:, ARM] == 0, axis=1))   # drop zero-config query frames

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    Bemb = torch.tensor(emb[bm], device=dev)
    fut_b, _ = build_future(act[bm], epi[bm], args.N)   # (nb, N, 26) neighbor future chunks
    fut_b_t = torch.tensor(fut_b, device=dev)

    # test frames in order
    q_ord = np.where(keepq)[0]
    Qemb = torch.tensor(emb[q_ord], device=dev)
    GT = act[q_ord]
    q_epi = epi[q_ord]

    # per-query retrieved chunk prediction: (nq, N, 26)
    @torch.no_grad()
    def retrieve_chunks():
        out = []
        for i in range(0, len(Qemb), 256):
            q = Qemb[i:i + 256]
            sims = q @ Bemb.T
            vals, idx = torch.topk(sims, args.k, dim=1)
            w = torch.softmax(vals / args.temp, dim=1)          # (b, k)
            neigh = fut_b_t[idx]                                 # (b, k, N, 26)
            out.append((w[:, :, None, None] * neigh).sum(1).cpu().numpy())
        return np.concatenate(out)                               # (nq, N, 26)

    pc = retrieve_chunks()
    nq = len(pc)

    # single-step baseline = chunk step 0
    l1_single = np.abs(pc[:, 0, :] - GT)

    # temporal ensembling: ensemble[t] = sum_d w_d * pc[t-d, d]  (t-d same test episode)
    wd = np.exp(-args.ens_m * np.arange(args.N))                 # weight by chunk offset
    ens = np.zeros_like(GT)
    wsum = np.zeros((nq, 1), np.float32)
    for d in range(args.N):
        src = np.arange(nq) - d
        ok = (src >= 0)
        srcc = np.clip(src, 0, nq - 1)
        same = ok & (q_epi[srcc] == q_epi)                      # t-d is same test episode & contiguous
        contrib = pc[srcc, d, :]                                # prediction for frame t from query t-d
        ens += np.where(same[:, None], wd[d] * contrib, 0.0)
        wsum += np.where(same[:, None], wd[d], 0.0)
    ens = ens / np.maximum(wsum, 1e-8)
    l1_ens = np.abs(ens - GT)

    print(f"held-out {n_test} eps | test frames {nq} | k={args.k} N={args.N} ens_m={args.ens_m}")
    print(f"{'method':<22}{'L1 overall':>11}{'L1 arm':>9}{'L1 hand':>9}")
    print(f"{'single-step (N=1)':<22}{l1_single.mean():>11.4f}{l1_single[:, ARM].mean():>9.4f}{l1_single[:, HAND].mean():>9.4f}")
    print(f"{'chunk+ensembling':<22}{l1_ens.mean():>11.4f}{l1_ens[:, ARM].mean():>9.4f}{l1_ens[:, HAND].mean():>9.4f}")


if __name__ == "__main__":
    main()
