# Multi-Agent Path Finding for Earliness-Tardiness Costs
code for paper "just-in-time multi agent path finding for earliness-tardiness costs"
The 2025 IEEE International Conference on Real-time Computing and Robotics
## Usage
```
sudo apt install libboost-all-dev
cmake -DCMAKE_BUILD_TYPE=RELEASE .
make
./build/cbs_et -m benchmark/maps/random-32-32-20.map -a benchmark/mapf-dt-scens/random-32-32-20/random-32-32-20-tc-random-1.scen -p test.csv --outputPaths=paths.txt -n 50 -t 60 -s 1 --addend=0 --objective=0
./build/cbs_et --help
python Simulator.py -m benchmark/maps/random-32-32-20.map -p paths.txt
```
## Reference
[1] Weibin Ye, Hanfu Wang, Weidong Chen. Just-In-Time Multi-Agent Path Finding with Earliness-Tardiness Costs[C]. IEEE International Conference on Real-time Computing and Robotics. Toyama, Japan, June 1- 6, 2025.

[2] Hanfu Wang, Weidong Chen. Multi-robot path planning with due times[J]. IEEE Robotics and Automation Letters, 2022, 7(2): 4829-4836.

[3] Jiaoyang Li, Wheeler Ruml and Sven Koenig. EECBS: Bounded-Suboptimal Search for Multi-Agent Path Finding. In Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), pages 12353-12362, 2021.
