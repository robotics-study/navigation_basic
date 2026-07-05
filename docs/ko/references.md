---
title: 참고 문헌
layout: default
---

# 참고 문헌

프로젝트 전체에서 인용하는 문헌. 각 항목은 공식 페이지(**DOI**/출판사)와, 무료로 PDF 를 내려받을 수
있는 곳이 있으면 **PDF** 링크를 함께 제공한다. 각 알고리즘 페이지 하단 각주와 동일한 출처다.

## 구현된 알고리즘

- **BFS** — Moore, E. F. (1959). "The shortest path through a maze." *Proc. Int. Symp. on the Theory of Switching*, Harvard University Press, 285–292.
  - 배선 문제 독립 발견: Lee, C. Y. (1961). "An algorithm for path connections and its applications." *IRE Trans. Electronic Computers*, EC-10(3), 346–365. — [DOI](https://doi.org/10.1109/TEC.1961.5219222)
- **Dijkstra** — Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs." *Numerische Mathematik*, 1, 269–271. — [DOI](https://doi.org/10.1007/BF01386390)
- **A\*** — Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Trans. Systems Science and Cybernetics*, 4(2), 100–107. — [DOI](https://doi.org/10.1109/TSSC.1968.300136)
  - 정정: Hart, Nilsson & Raphael (1972). *SIGART Newsletter*, 37, 28–29. — [DOI](https://doi.org/10.1145/1056777.1056779)
  - Weighted A\*: Pohl, I. (1970). "Heuristic search viewed as path finding in a graph." *Artificial Intelligence*, 1(3–4), 193–204. — [DOI](https://doi.org/10.1016/0004-3702%2870%2990007-X)
- **RRT** — LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning." Technical Report TR 98-11, Iowa State University. — [PDF](https://lavalle.pl/papers/Lav98c.pdf)
  - Kinodynamic: LaValle, S. M., & Kuffner, J. J. (2001). "Randomized kinodynamic planning." *Int. J. of Robotics Research*, 20(5), 378–400. — [DOI](https://doi.org/10.1177/02783640122067453) · [PDF](https://lavalle.pl/papers/LavKuf01b.pdf)
- **RRT\*** — Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *Int. J. of Robotics Research*, 30(7), 846–894. — [DOI](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
- **Fast-RRT** — Wu, Z., Meng, Z., Zhao, W., & Wu, Z. (2021). "Fast-RRT: A RRT-Based Optimal Path Finding Method." *Applied Sciences*, 11(24), 11777. — [PDF (open access)](https://www.mdpi.com/2076-3417/11/24/11777) · [DOI](https://doi.org/10.3390/app112411777)

## 계획된 알고리즘

- **RRT-Connect** — Kuffner, J. J., & LaValle, S. M. (2000). "RRT-Connect: An efficient approach to single-query path planning." *Proc. IEEE ICRA*, 995–1001. — [DOI](https://doi.org/10.1109/ROBOT.2000.844730)
- **Informed RRT\*** — Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). "Informed RRT\*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic." *Proc. IEEE/RSJ IROS*, 2997–3004. — [DOI](https://doi.org/10.1109/IROS.2014.6942976) · [PDF (arXiv)](https://arxiv.org/abs/1404.2334)
- **DWA** — Fox, D., Burgard, W., & Thrun, S. (1997). "The dynamic window approach to collision avoidance." *IEEE Robotics & Automation Magazine*, 4(1), 23–33. — [DOI](https://doi.org/10.1109/100.580977)
- **Pure Pursuit** — Coulter, R. C. (1992). "Implementation of the Pure Pursuit path tracking algorithm." CMU-RI-TR-92-01, Carnegie Mellon University. — [PDF](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- **VFH** — Borenstein, J., & Koren, Y. (1991). "The vector field histogram — fast obstacle avoidance for mobile robots." *IEEE Trans. Robotics and Automation*, 7(3), 278–288. — [DOI](https://doi.org/10.1109/70.88137)
- **Prioritized planning** — Erdmann, M., & Lozano-Pérez, T. (1987). "On multiple moving objects." *Algorithmica*, 2, 477–521. — [DOI](https://doi.org/10.1007/BF01840371)
- **CBS** — Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015). "Conflict-based search for optimal multi-agent pathfinding." *Artificial Intelligence*, 219, 40–66. — [DOI](https://doi.org/10.1016/j.artint.2014.11.006)

## 배경

- **Occupancy grid** — Elfes, A. (1989). "Using occupancy grids for mobile robot perception and navigation." *Computer*, 22(6), 46–57. — [DOI](https://doi.org/10.1109/2.30720)
- **교과서** — LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press. — [무료 온라인](https://lavalle.pl/planning/)
- **MPC 교과서** — Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design* (2nd ed.). Nob Hill Publishing.

---

링크 표기: **DOI** = 출판사 공식 페이지(일부 유료). **PDF** = 저자/오픈액세스/프리프린트의 무료 다운로드
페이지. 유료 저널 논문은 DOI 만 제공한다.
