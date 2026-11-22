"""Check either side of the short-horizon neutral-target boundary over eight seconds."""
from concurrent.futures import ProcessPoolExecutor

from train_release_catch import WORK, batch, job, read, write

if __name__=='__main__':
    trims=read(WORK/'neutral-trim.json')['trims']
    jobs=[]
    for side in (-1,1):
        low,high=trims[str(side)]['initial_bracket_deg']
        midpoint=(low+high)/2.
        for dt in (.000125,.0001):
            for delta in (-.02,.02):
                jobs.append(job(side,midpoint+delta,0.,None,8.,96610,dt))
    with ProcessPoolExecutor(2) as pool:
        write(WORK/'neutral-verification.json',batch(pool,jobs))
