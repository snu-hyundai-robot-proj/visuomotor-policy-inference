"""VINN-style kNN policy — step 2: held-out L1 evaluation over the DINOv3 embedding bank.

Holds out ~10% of episodes as the query/test set, builds the kNN memory from the rest, and
for each test frame retrieves the k nearest demo embeddings (cosine) and returns the
locally-weighted average of their actions (LWR). Reports L1 vs ground-truth action.
"""
import argparse
import numpy as np
import torch

BANK = "/home/ngseo/visuomotor-policy-inference/knn_bank_left.npz"
ARM = slice(0, 6)
HAND = slice(6, 26)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bank", default=BANK)
    ap.add_argument("--holdout_frac", type=float, default=0.1)
    ap.add_argument("--temp", type=float, default=0.05)
    args = ap.parse_args()

    z = np.load(args.bank)
    emb, act, epi = z["emb"], z["act"], z["epi"]
    eps = np.unique(epi)
    n_test = max(1, int(len(eps) * args.holdout_frac))
    test_eps = set(eps[:n_test].tolist())
    tm = np.isin(epi, list(test_eps))
    bm = ~tm
    # drop leading zero-config arm frames (recording artifact) from the query set
    keep = ~np.all(act[:, ARM] == 0, axis=1)
    qm = tm & keep
    print(f"bank frames: {bm.sum()} (episodes {len(eps)-n_test}) | test frames: {qm.sum()} "
          f"(held-out episodes {n_test})")

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    B = torch.tensor(emb[bm], device=dev)        # (nb, 1536) unit-norm
    Ba = torch.tensor(act[bm], device=dev)       # (nb, 26)
    Q = torch.tensor(emb[qm], device=dev)
    GT = act[qm]

    @torch.no_grad()
    def knn_pred(k, temp):
        out = []
        for i in range(0, len(Q), 512):
            q = Q[i:i + 512]
            sims = q @ B.T                       # cosine similarity (unit-norm)
            vals, idx = torch.topk(sims, k, dim=1)
            w = torch.softmax(vals / temp, dim=1)                 # (b, k) LWR weights
            neigh = Ba[idx]                       # (b, k, 26)
            out.append((w.unsqueeze(-1) * neigh).sum(1).cpu().numpy())
        return np.concatenate(out)

    print(f"\n=== kNN (VINN) L1 on held-out episodes  (temp={args.temp}) ===")
    print(f"{'k':>4} {'L1 overall':>11} {'L1 arm':>9} {'L1 hand':>9}")
    for k in [1, 4, 8, 16, 32, 64]:
        pred = knn_pred(k, args.temp)
        l1 = np.abs(pred - GT)
        print(f"{k:>4} {l1.mean():>11.4f} {l1[:, ARM].mean():>9.4f} {l1[:, HAND].mean():>9.4f}")


if __name__ == "__main__":
    main()
