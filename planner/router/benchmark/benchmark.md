## Compare results

### graph_bm
#### x86
Use the origin graph with string vertex
``` shell
-----------------------------------------------------------------------
Benchmark                             Time             CPU   Iterations
-----------------------------------------------------------------------
BM_StringVertex/5                 0.044 ms        0.044 ms        11953
BM_StringFormatVertex/5           0.019 ms        0.019 ms        37003
BM_IntVertex/5                    0.001 ms        0.001 ms       490661
BM_StringGraph/5                  0.204 ms        0.204 ms         3377
BM_StringGraphDijkstra/5          0.086 ms        0.086 ms         8220
BM_BoostInt64GraphDijkstra/5      0.029 ms        0.029 ms        24852
```
#### j5
``` shell
Running /userdata/benchmark/graph_bm
Run on (8 X 1200 MHz CPU s)
Load Average: 4.08, 4.11, 4.14
-----------------------------------------------------------------------
Benchmark                             Time             CPU   Iterations
-----------------------------------------------------------------------
BM_StringVertex/5                 0.410 ms        0.408 ms         1732
BM_StringFormatVertex/5           0.160 ms        0.159 ms         4899
BM_IntVertex/5                    0.010 ms        0.010 ms        59947
BM_StringGraph/5                   1.97 ms         1.96 ms          357
BM_StringGraphDijkstra/5          0.767 ms        0.764 ms          910
BM_BoostInt64GraphDijkstra/5      0.251 ms        0.250 ms         2744
```

#### 20230715
------------------------------------------------------------------
Benchmark                        Time             CPU   Iterations
------------------------------------------------------------------
BM_CalcCurRouteNaviInfo       2.96 ms         2.96 ms          256

-----------------------------------------------------------------------
Benchmark                             Time             CPU   Iterations
-----------------------------------------------------------------------
BM_StringVertex/5                 0.050 ms        0.050 ms        12753
BM_StringFormatVertex/5           0.021 ms        0.021 ms        29055
BM_IntVertex/5                    0.002 ms        0.002 ms       344369
BM_BoostInt64GraphDijkstra/5      0.021 ms        0.021 ms        32333
