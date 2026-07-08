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
- **ARA\*** — Likhachev, M., Gordon, G., & Thrun, S. (2003). "ARA\*: Anytime A\* with Provable Bounds on Sub-Optimality." *Advances in Neural Information Processing Systems (NIPS)* 16. — [PDF](https://papers.nips.cc/paper/2003/hash/ee8fe9093fbbb687bef15a38facc44d2-Abstract.html)
- **AD\*** — Likhachev, M., Ferguson, D., Gordon, G., Stentz, A., & Thrun, S. (2005). "Anytime Dynamic A\*: An Anytime, Replanning Algorithm." *Proc. ICAPS*, 262–271. — [PDF](https://www.cs.cmu.edu/~maxim/files/ad_icaps05.pdf)
  - Weighted A\*: Pohl, I. (1970). "Heuristic search viewed as path finding in a graph." *Artificial Intelligence*, 1(3–4), 193–204. — [DOI](https://doi.org/10.1016/0004-3702%2870%2990007-X)
- **JPS** — Harabor, D., & Grastien, A. (2011). "Online Graph Pruning for Pathfinding on Grid Maps." *Proc. AAAI Conference on Artificial Intelligence*, 1114–1119. — [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/7994)
- **D\* Lite** — Koenig, S., & Likhachev, M. (2002). "D\* Lite." *Proc. AAAI Conference on Artificial Intelligence*, 476–483. — [PDF](https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf)
  - 기반 LPA\*: Koenig, S., Likhachev, M., & Furcy, D. (2004). "Lifelong Planning A\*." *Artificial Intelligence*, 155(1–2), 93–146. — [DOI](https://doi.org/10.1016/j.artint.2003.12.001)
- **Theta\*** — Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). "Theta\*: Any-Angle Path Planning on Grids." *Proc. AAAI Conference on Artificial Intelligence*, 1177–1183. — [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/11009)
  - LOS supercover: Amanatides, J., & Woo, A. (1987). "A Fast Voxel Traversal Algorithm for Ray Tracing." *Proc. Eurographics*, 3–10. — [PDF](https://www.cse.yorku.ca/~amana/research/grid.pdf)
- **Lazy Theta\*** — Nash, A., Koenig, S., & Tovey, C. (2010). "Lazy Theta\*: Any-Angle Path Planning and Path Length Analysis in 3D." *Proc. AAAI Conference on Artificial Intelligence*, 147–154. — [PDF](http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf)
- **Anya** — Harabor, D., Grastien, A., Öz, D., & Aksakalli, V. (2016). "Optimal Any-Angle Pathfinding In Practice." *Journal of Artificial Intelligence Research (JAIR)*, 56, 89–118. — [DOI](https://doi.org/10.1613/jair.5007)
- **Visibility A\*** — visibility-graph 최단 경로: Lozano-Pérez, T., & Wesley, M. A. (1979). "An algorithm for planning collision-free paths among polyhedral obstacles." *Communications of the ACM*, 22(10), 560–570. — [DOI](https://doi.org/10.1145/359156.359164) (이 저장소 변형은 cell-centre visibility graph 위 A\*; 셀-중심 근사로 참 Euclidean any-angle 최적은 아님)
- **Hybrid A\*** — Dolgov, D., Thrun, S., Montemerlo, M., & Diebel, J. (2008). "Practical Search Techniques in Path Planning for Autonomous Driving." *Proc. STAIR (AAAI Workshop)*. — [PDF](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
  - Non-holonomic distance heuristics: Reeds, J. A., & Shepp, L. A. (1990). "Optimal paths for a car that goes both forwards and backwards." *Pacific J. of Mathematics*, 145(2), 367–393. — [DOI](https://doi.org/10.2140/pjm.1990.145.367)
- **RRT** — LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning." Technical Report TR 98-11, Iowa State University. — [PDF](https://lavalle.pl/papers/Lav98c.pdf)
  - Kinodynamic: LaValle, S. M., & Kuffner, J. J. (2001). "Randomized kinodynamic planning." *Int. J. of Robotics Research*, 20(5), 378–400. — [DOI](https://doi.org/10.1177/02783640122067453) · [PDF](https://lavalle.pl/papers/LavKuf01b.pdf)
- **RRT-Connect** — Kuffner, J. J., & LaValle, S. M. (2000). "RRT-Connect: An efficient approach to single-query path planning." *Proc. IEEE ICRA*, 995–1001. — [DOI](https://doi.org/10.1109/ROBOT.2000.844730)
- **RRT\*** — Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *Int. J. of Robotics Research*, 30(7), 846–894. — [DOI](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
- **LQR-RRT\*** — Perez, A., Platt, R., Konidaris, G., Kaelbling, L., & Lozano-Pérez, T. (2012). "LQR-RRT\*: Optimal Sampling-Based Motion Planning with Automatically Derived Extension Heuristics." *Proc. IEEE ICRA*, 2537–2542. — [DOI](https://doi.org/10.1109/ICRA.2012.6225177) · [PDF](https://lis.csail.mit.edu/pubs/perez-icra12.pdf)
- **Kinodynamic RRT\*** — Webb, D. J., & van den Berg, J. (2013). "Kinodynamic RRT\*: Asymptotically Optimal Motion Planning for Robots with Linear Dynamics." *Proc. IEEE ICRA*, 5054–5061. — [DOI](https://doi.org/10.1109/ICRA.2013.6631299) · [PDF (arXiv)](https://arxiv.org/abs/1205.5088)
- **Informed RRT\*** — Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). "Informed RRT\*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic." *Proc. IEEE/RSJ IROS*, 2997–3004. — [DOI](https://doi.org/10.1109/IROS.2014.6942976) · [PDF (arXiv)](https://arxiv.org/abs/1404.2334)
- **PRM** — Kavraki, L. E., Švestka, P., Latombe, J.-C., & Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces." *IEEE Trans. Robotics and Automation*, 12(4), 566–580. — [DOI](https://doi.org/10.1109/70.508439)
- **PRM\*** — Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *Int. J. of Robotics Research*, 30(7), 846–894. — [DOI](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
- **FMT\*** — Janson, L., Schmerling, E., Clark, A., & Pavone, M. (2015). "Fast marching tree: A fast marching sampling-based method for optimal motion planning in many dimensions." *Int. J. of Robotics Research*, 34(7), 883–921. — [DOI](https://doi.org/10.1177/0278364915577958) · [PDF (arXiv)](https://arxiv.org/abs/1306.3532)
- **BIT\*** — Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2015). "Batch Informed Trees (BIT\*): Sampling-based optimal planning via the heuristically guided search of implicit random geometric graphs." *Proc. IEEE ICRA*, 3067–3074. — [DOI](https://doi.org/10.1109/ICRA.2015.7139620) · [PDF (arXiv)](https://arxiv.org/abs/1405.5848)
- **ABIT\*** — Strub, M. P., & Gammell, J. D. (2020). "Advanced BIT\* (ABIT\*): Sampling-Based Planning with Advanced Graph-Search Techniques." *Proc. IEEE ICRA*, 130–136. — [DOI](https://doi.org/10.1109/ICRA40945.2020.9196580) · [PDF (arXiv)](https://arxiv.org/abs/2002.06589)
  - Informed sampling (타원): Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). "Informed RRT\*..." *Proc. IEEE/RSJ IROS*, 2997–3004. — [DOI](https://doi.org/10.1109/IROS.2014.6942976) · [PDF (arXiv)](https://arxiv.org/abs/1404.2334)
- **SST / SST\*** — Li, Y., Littlefield, Z., & Bekris, K. E. (2016). "Asymptotically optimal sampling-based kinodynamic planning." *Int. J. of Robotics Research*, 35(5), 528–564. — [DOI](https://doi.org/10.1177/0278364915614386) · [PDF (arXiv)](https://arxiv.org/abs/1407.2896)
- **AIT\*** — Strub, M. P., & Gammell, J. D. (2020). "Adaptively Informed Trees (AIT\*): Fast Asymptotically Optimal Path Planning through Adaptive Heuristics." *Proc. IEEE ICRA*, 3191–3198. — [DOI](https://doi.org/10.1109/ICRA40945.2020.9197338) · [PDF (arXiv)](https://arxiv.org/abs/2002.06599)
- **EIT\*** — Strub, M. P., & Gammell, J. D. (2022). "Adaptively Informed Trees (AIT\*) and Effort Informed Trees (EIT\*): Asymmetric bidirectional sampling-based path planning." *Int. J. of Robotics Research*, 41(4), 390–417. — [DOI](https://doi.org/10.1177/02783649211069572) · [PDF (arXiv)](https://arxiv.org/abs/2111.01877)
- **FCIT\*** — Wilson, T., Strub, M. P., & Gammell, J. D. (2025). "Nearest-Neighbourless Asymptotically Optimal Motion Planning with Fully Connected Informed Trees (FCIT\*)." *Proc. IEEE ICRA*. — [PDF (arXiv)](https://arxiv.org/abs/2411.17902)
- **Fast-RRT** — Wu, Z., Meng, Z., Zhao, W., & Wu, Z. (2021). "Fast-RRT: A RRT-Based Optimal Path Finding Method." *Applied Sciences*, 11(24), 11777. — [PDF (open access)](https://www.mdpi.com/2076-3417/11/24/11777) · [DOI](https://doi.org/10.3390/app112411777)

## 계획된 알고리즘

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
