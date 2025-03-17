
### Building:
```shell
cd RRTSTAR
make OPTIONS=“-DXXX”
```
XXX可以有下面任何一個
FIXED_SEED 固定亂數種
TEST_TIME 跑十次取平均
UPDATE_ONCE 只更新一次圖，對策量沒影響，但可以跑快點
PARALLEL 第一種平行化
PARALLEL2 第二種

### Running the RRT* algorithm:
```shell
cd RRTSTAR
./bin/RRTSTAR ./XXX.txt
```
XXX is the file under RRTSTAR,you can choose one of them
After run the program,you can check the test.bmp to see the map and path

[1] S. Karaman and E. Frazzoli, “Incremental sampling-based algorithms for optimal motion planning,” in Proc. Robotics: Science and Systems (RSS), 2010
