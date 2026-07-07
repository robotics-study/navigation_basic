---
title: 알고리즘
layout: default
parent: 한국어
nav_order: 1
has_children: true
permalink: /ko/algorithms/
---

[🇰🇷 한국어](index.md) | [🇬🇧 English](../../en/algorithms/index.md)

# 알고리즘

구현된 알고리즘별 상세 페이지. 각 페이지는 이론적 배경(원 논문 각주 포함), pseudocode,
성질(완전성·최적성·복잡도), 파라미터 선언, 방출 trace 이벤트, demo 시각화(GIF + 중간 과정 PNG)와
실측 metric 을 담는다.

## Global planning — 구현 완료

| 알고리즘 | 연도 | 접근 | 완전성 | 최적성 | 원 논문 |
|---|---|---|---|---|---|
| [BFS](bfs.md) | 1959 | uninformed search | complete | edge 수 최소 (unit cost 에서 최적) | Moore [^moore] |
| [Dijkstra](dijkstra.md) | 1959 | uninformed search | complete | cost 최적 | Dijkstra [^dijkstra] |
| [A*](astar.md) | 1968 | informed search | complete | cost 최적 (admissible h) | Hart, Nilsson & Raphael [^hart] |
| [JPS](jps.md) | 2011 | grid 대칭 제거 | complete | cost 최적 (= 8-connected A*) | Harabor & Grastien [^harabor_jps] |
| [D\* Lite](dstar_lite.md) | 2002 | dynamic replanning | complete | belief 기준 최적 (증분 수리) | Koenig & Likhachev [^koenig] |
| [Theta\*](theta_star.md) | 2007 | any-angle search | complete | any-angle (grid-optimal 아님) | Nash, Daniel, Koenig & Felner [^nash] |
| [Lazy Theta\*](lazy_theta_star.md) | 2010 | any-angle search (lazy LOS) | complete | any-angle (grid-optimal 아님) | Nash, Koenig & Tovey [^nash_lazy] |
| [Visibility A\*](visibility_astar.md) | — | any-angle (visibility graph) | complete | cell-centre visibility 최단 (참 Euclidean 최적 아님) | — |
| [Hybrid A\*](hybrid_astar.md) | 2008 | kinodynamic search | resolution-complete | resolution-suboptimal (feasible) | Dolgov, Thrun, Montemerlo & Diebel [^dolgov] |
| [RRT](rrt.md) | 1998 | sampling | probabilistically complete | 비최적 (feasible) | LaValle [^lavalle98] |
| [RRT-Connect](rrt_connect.md) | 2000 | sampling (양방향) | probabilistically complete | 비최적 (feasible, single-query) | Kuffner & LaValle [^kuffner] |
| [RRT\*](rrt_star.md) | 2011 | sampling | probabilistically complete | asymptotically optimal | Karaman & Frazzoli [^karaman] |
| [Kinodynamic RRT\*](kinodynamic_rrt_star.md) | 2013 | sampling (kinodynamic) | probabilistically complete | asymptotically optimal (동역학 비용) | Webb & van den Berg [^webb] |
| [Informed RRT\*](informed_rrt_star.md) | 2014 | sampling (direct informed) | probabilistically complete | asymptotically optimal | Gammell, Srinivasa & Barfoot [^gammell] |
| [PRM](prm.md) | 1996 | sampling (roadmap) | probabilistically complete | 비최적 (고정 반경) | Kavraki et al. [^kavraki] |
| [PRM\*](prm_star.md) | 2011 | sampling (roadmap) | probabilistically complete | asymptotically optimal | Karaman & Frazzoli [^karaman] |
| [FMT\*](fmt_star.md) | 2015 | sampling (batch marching) | probabilistically complete | asymptotically optimal | Janson et al. [^janson] |
| [BIT\*](bit_star.md) | 2015 | sampling (batch + informed) | probabilistically complete | almost-surely asymptotically optimal | Gammell et al. [^gammell_bit] |
| [SST](sst.md) | 2016 | sampling (kinodynamic, forward-propagation) | probabilistically complete | near-optimal (SST) / asymptotically optimal (SST\*) | Li, Littlefield & Bekris [^li] |
| [AIT\*](ait_star.md) | 2020 | sampling (비대칭 양방향) | probabilistically complete | almost-surely asymptotically optimal | Strub & Gammell [^strub_ait] |
| [EIT\*](eit_star.md) | 2022 | sampling (effort-informed 양방향) | probabilistically complete | almost-surely asymptotically optimal | Strub & Gammell [^strub_eit] |
| [FCIT\*](fcit_star.md) | 2025 | sampling (완전 연결 informed) | probabilistically complete | almost-surely asymptotically optimal | Wilson, Strub & Gammell [^wilson_fcit] |
| [Fast-RRT](fast_rrt.md) | 2021 | sampling (RRT\* 계열 변형) | probabilistically complete | near-optimal (RRT\* 대비 고속 수렴) | Wu et al. [^wu] |

> **계보 메모.** sampling 최적 계열은 두 흐름이 모두 informed·batch·양방향 탐색으로 수렴한다.
> **단일 트리 최적:** RRT\* (2011) → Informed RRT\* (2014, 타원체 direct sampling). **로드맵/배치
> 최적:** PRM → PRM\* → FMT\* → BIT\* (2015). 이 둘은 **informed-trees 계열** BIT\* → AIT\* (2020,
> 적응적 역방향 휴리스틱을 쓰는 비대칭 양방향) → EIT\* (2022, effort-informed) → FCIT\* (2025, 값싼
> 충돌검사를 활용한 완전 연결)로 합류한다 — 현재 점근 최적의 최전선이다. 별개로 **RRT-Connect**
> (2000)는 RRT 의 양방향 single-query 후손이고, [Fast-RRT](fast_rrt.md) (2021)는 shortcut 으로
> 수렴을 앞당긴 RRT\* 변형이다. kinodynamic sampling 갈래에서는 **Kinodynamic RRT\*** (2013)가 최적
> steering controller 로 RRT\* 의 최적성을 미분 제약 시스템까지 확장하고, **SST** (2016)는 witness
> 집합으로 트리를 sparse 하게 유지해 steering/BVP 없이 forward propagation 만으로 점근 최적을 달성한다.
>
> **grid 탐색** 계열도 갈라진다. **JPS** (2011)는 A\* 의 무손실 successor pruning 으로 균일비용
> 8-connected grid 에서 동일한 최적 경로를 확장 수를 한 자릿수 줄여 반환하고, **any-angle** 계열은
> Theta\* (2007) → **Lazy Theta\*** (2010, LOS 검사를 간선당이 아니라 확장 정점당 1회로) →
> **Visibility A\***로 이어지며, 확장된 root 에서 LOS-가시한 모든 셀을 relax 해 셀-중심 visibility graph
> 위 최단을 얻는다 — Theta\* 를 지배하지만 회전점이 셀 중심에 고정된 근사다(참 Euclidean 최적 아님).
> 소스는 평탄한 디렉토리에 두되, 위 트리는 폴더 구조가 아니라 계보다.

## 계획 (미구현)

| 카테고리 | 알고리즘 | 원 논문 |
|---|---|---|
| local_planning | DWA | Fox, Burgard & Thrun (1997) [^fox] |
| local_planning | Pure Pursuit | Coulter (1992) [^coulter] |
| local_planning | VFH | Borenstein & Koren (1991) [^borenstein] |
| local_planning | MPC | — (survey: Rawlings, Mayne & Diehl 2017) |
| multi_agent | Prioritized planning | Erdmann & Lozano-Pérez (1987) [^erdmann] |
| multi_agent | CBS | Sharon et al. (2015) [^sharon] |

## 공통 실행 방법

모든 데모는 두 언어에서 동일한 CLI 인자를 갖는다:

```bash
# Python
python python/demos/demo_<algo>.py \
  --map maps/grid/<map>.yaml --scenario maps/scenarios/<map>_s1.yaml \
  --params configs/global_planning/<algo>.yaml --trace out/<algo>.jsonl \
  [--seed N] [--connectivity 4|8]

# C++ (동일 인자)
./cpp/build/demos/demo_<algo> --map ... --scenario ... --params ... --trace ...

# 시각화: GIF 애니메이션 + 탐색 중간 과정 PNG 스냅샷
python tools/viz/replay.py out/<algo>.jsonl --gif out/<algo>.gif --snapshots out/snaps/
```

## References

[^moore]: Moore, E. F. (1959). "The shortest path through a maze." *Proceedings of the International Symposium on the Theory of Switching*, Harvard University Press, 285–292.
[^dijkstra]: Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs." *Numerische Mathematik*, 1, 269–271. [doi:10.1007/BF01386390](https://doi.org/10.1007/BF01386390)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
[^nash]: Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). "Theta\*: Any-Angle Path Planning on Grids." *Proc. AAAI Conference on Artificial Intelligence*, 1177–1183. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/11009)
[^koenig]: Koenig, S., & Likhachev, M. (2002). "D\* Lite." *Proc. AAAI Conference on Artificial Intelligence*, 476–483. [PDF](https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf)
[^dolgov]: Dolgov, D., Thrun, S., Montemerlo, M., & Diebel, J. (2008). "Practical Search Techniques in Path Planning for Autonomous Driving." *Proc. STAIR (AAAI Workshop)*. [PDF](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
[^lavalle98]: LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning." Technical Report TR 98-11, Computer Science Dept., Iowa State University. [PDF](https://lavalle.pl/papers/Lav98c.pdf)
[^karaman]: Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *The International Journal of Robotics Research*, 30(7), 846–894. [doi:10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
[^kavraki]: Kavraki, L. E., Švestka, P., Latombe, J.-C., & Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces." *IEEE Transactions on Robotics and Automation*, 12(4), 566–580. [doi:10.1109/70.508439](https://doi.org/10.1109/70.508439)
[^janson]: Janson, L., Schmerling, E., Clark, A., & Pavone, M. (2015). "Fast marching tree: A fast marching sampling-based method for optimal motion planning in many dimensions." *The International Journal of Robotics Research*, 34(7), 883–921. [doi:10.1177/0278364915577958](https://doi.org/10.1177/0278364915577958) · [PDF (arXiv)](https://arxiv.org/abs/1306.3532)
[^gammell_bit]: Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2015). "Batch Informed Trees (BIT\*): Sampling-based optimal planning via the heuristically guided search of implicit random geometric graphs." *Proc. IEEE ICRA*, 3067–3074. [doi:10.1109/ICRA.2015.7139620](https://doi.org/10.1109/ICRA.2015.7139620) · [PDF (arXiv)](https://arxiv.org/abs/1405.5848)
[^wu]: Wu, Z., Meng, Z., Zhao, W., & Wu, Z. (2021). "Fast-RRT: A RRT-Based Optimal Path Finding Method." *Applied Sciences*, 11(24), 11777. [doi:10.3390/app112411777](https://doi.org/10.3390/app112411777) · [PDF (open access)](https://www.mdpi.com/2076-3417/11/24/11777)
[^strub_ait]: Strub, M. P., & Gammell, J. D. (2020). "Adaptively Informed Trees (AIT\*): Fast Asymptotically Optimal Path Planning through Adaptive Heuristics." *Proc. IEEE ICRA*, 3191–3198. [doi:10.1109/ICRA40945.2020.9197338](https://doi.org/10.1109/ICRA40945.2020.9197338) · Extended in Strub & Gammell (2022), *IJRR* 41(4), 390–417. [PDF (arXiv)](https://arxiv.org/abs/2111.01877)
[^strub_eit]: Strub, M. P., & Gammell, J. D. (2022). "Adaptively Informed Trees (AIT\*) and Effort Informed Trees (EIT\*): Asymmetric bidirectional sampling-based path planning." *The International Journal of Robotics Research*, 41(4), 390–417. [doi:10.1177/02783649211069572](https://doi.org/10.1177/02783649211069572) · [PDF (arXiv)](https://arxiv.org/abs/2111.01877)
[^wilson_fcit]: Wilson, T., Strub, M. P., & Gammell, J. D. (2025). "Nearest-Neighbourless Asymptotically Optimal Motion Planning with Fully Connected Informed Trees (FCIT\*)." *Proc. IEEE ICRA*. [PDF (arXiv:2411.17902)](https://arxiv.org/abs/2411.17902)
[^harabor_jps]: Harabor, D., & Grastien, A. (2011). "Online Graph Pruning for Pathfinding on Grid Maps." *Proc. AAAI Conference on Artificial Intelligence*, 1114–1119. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/7994)
[^nash_lazy]: Nash, A., Koenig, S., & Tovey, C. (2010). "Lazy Theta\*: Any-Angle Path Planning and Path Length Analysis in 3D." *Proc. AAAI Conference on Artificial Intelligence*, 147–154. [PDF](http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf)
[^webb]: Webb, D. J., & van den Berg, J. (2013). "Kinodynamic RRT\*: Asymptotically Optimal Motion Planning for Robots with Linear Dynamics." *Proc. IEEE ICRA*, 5054–5061. [doi:10.1109/ICRA.2013.6631299](https://doi.org/10.1109/ICRA.2013.6631299) · [PDF (arXiv)](https://arxiv.org/abs/1205.5088)
[^li]: Li, Y., Littlefield, Z., & Bekris, K. E. (2016). "Asymptotically optimal sampling-based kinodynamic planning." *The International Journal of Robotics Research*, 35(5), 528–564. [doi:10.1177/0278364915614386](https://doi.org/10.1177/0278364915614386) · [PDF (arXiv)](https://arxiv.org/abs/1407.2896)
[^kuffner]: Kuffner, J. J., & LaValle, S. M. (2000). "RRT-Connect: An efficient approach to single-query path planning." *Proc. IEEE ICRA*, 995–1001. [doi:10.1109/ROBOT.2000.844730](https://doi.org/10.1109/ROBOT.2000.844730)
[^gammell]: Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). "Informed RRT\*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic." *Proc. IEEE/RSJ IROS*, 2997–3004. [doi:10.1109/IROS.2014.6942976](https://doi.org/10.1109/IROS.2014.6942976) · [PDF (arXiv)](https://arxiv.org/abs/1404.2334)
[^fox]: Fox, D., Burgard, W., & Thrun, S. (1997). "The dynamic window approach to collision avoidance." *IEEE Robotics & Automation Magazine*, 4(1), 23–33. [doi:10.1109/100.580977](https://doi.org/10.1109/100.580977)
[^coulter]: Coulter, R. C. (1992). "Implementation of the Pure Pursuit path tracking algorithm." Technical Report CMU-RI-TR-92-01, Robotics Institute, Carnegie Mellon University. [PDF](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
[^borenstein]: Borenstein, J., & Koren, Y. (1991). "The vector field histogram — fast obstacle avoidance for mobile robots." *IEEE Transactions on Robotics and Automation*, 7(3), 278–288. [doi:10.1109/70.88137](https://doi.org/10.1109/70.88137)
[^erdmann]: Erdmann, M., & Lozano-Pérez, T. (1987). "On multiple moving objects." *Algorithmica*, 2, 477–521. [doi:10.1007/BF01840371](https://doi.org/10.1007/BF01840371)
[^sharon]: Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015). "Conflict-based search for optimal multi-agent pathfinding." *Artificial Intelligence*, 219, 40–66. [doi:10.1016/j.artint.2014.11.006](https://doi.org/10.1016/j.artint.2014.11.006)
