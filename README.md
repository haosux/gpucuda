# gpucuda
| 算法 (毫秒)        |  1165G7 (Win) |  1165G7 (Linux) | RTX2080 Ti  8700 (Linux) | A770 i7-12700 (Linux)  | 
|------              | --------------|-----------------|---------------------|-------------------|
| pcl radius search  |  228          |    249          | 454                |   115              |    
| gpu radius search  |  13.8         |    9.1          | 3.32               |   4.35            |  
| pcl approx nearest |  8.7          |    8.7          | 14.3               |   5.32             |  
| gpu approx nearest |  0.36         |    0.124        | **0.047**          |   **0.37**             |  

pcl gpu 最邻近搜索这个算法，arc a770 远远落后 RTX2080Ti，这个问题有待调查差距太大的原因。


|算法            |  1165G7 (Win) 加速比 |  1165G7 (Linux) 加速比 | RTX2080 Ti 加速比 | A770(Linux) 加速比 |
|-----           |---------------------| ----------------------|------------------|------------------|
|radius          |16.52                |      27.4             |     136.7         |   26.4          |
|approx nearest  |24.2                 |      70.2             |     304.3         |   14.3          |





running Example  on i7-8700 and RTX 2080Ti 
```bash
cuda approx nearest took 0.249 milliseconds 
cuda approx nearest took 0.051 milliseconds 
cuda approx nearest took 0.050 milliseconds 
cuda approx nearest took 0.049 milliseconds 
cuda approx nearest took 0.050 milliseconds 
cuda approx nearest took 0.056 milliseconds 
cuda approx nearest took 0.051 milliseconds 
cuda approx nearest took 0.047 milliseconds 
cuda approx nearest took 0.045 milliseconds 
cuda approx nearest took 0.045 milliseconds 
cuda approx nearest took 0.045 milliseconds 
cuda approx nearest took 0.045 milliseconds 
cuda approx nearest took 0.059 milliseconds 
cuda approx nearest took 0.045 milliseconds 
cuda approx nearest took 0.045 milliseconds 
cuda approx nearest took 0.044 milliseconds 
cuda approx nearest took 0.045 milliseconds 
cuda approx nearest took 0.044 milliseconds 
cuda approx nearest took 0.044 milliseconds 
cuda approx nearest took 0.045 milliseconds 
result size is 10000 



[!] Host octree resolution: 25

pcl approx nearest took 14.969 milliseconds 
pcl approx nearest took 14.513 milliseconds 
pcl approx nearest took 15.121 milliseconds 
pcl approx nearest took 14.732 milliseconds 
pcl approx nearest took 14.495 milliseconds 
pcl approx nearest took 14.755 milliseconds 
pcl approx nearest took 13.943 milliseconds 
pcl approx nearest took 15.118 milliseconds 
pcl approx nearest took 14.274 milliseconds 
pcl approx nearest took 14.795 milliseconds 
pcl approx nearest took 13.933 milliseconds 
pcl approx nearest took 15.711 milliseconds 
pcl approx nearest took 14.243 milliseconds 
pcl approx nearest took 14.890 milliseconds 
pcl approx nearest took 14.338 milliseconds 
pcl approx nearest took 14.471 milliseconds 
pcl approx nearest took 14.380 milliseconds 
pcl approx nearest took 14.605 milliseconds 
pcl approx nearest took 14.441 milliseconds 
pcl approx nearest took 14.025 milliseconds 


cuda radius search took 2.910 milliseconds 
cuda radius search took 2.765 milliseconds 
cuda radius search took 3.322 milliseconds 
cuda radius search took 3.044 milliseconds 
cuda radius search took 2.766 milliseconds 
cuda radius search took 2.766 milliseconds 
cuda radius search took 2.767 milliseconds 
cuda radius search took 4.067 milliseconds 
cuda radius search took 4.900 milliseconds 
cuda radius search took 3.042 milliseconds 
cuda radius search took 2.773 milliseconds 
cuda radius search took 2.763 milliseconds 
cuda radius search took 2.767 milliseconds 
cuda radius search took 3.085 milliseconds 
cuda radius search took 2.774 milliseconds 
cuda radius search took 2.773 milliseconds 
cuda radius search took 2.765 milliseconds 
cuda radius search took 4.202 milliseconds 
cuda radius search took 4.318 milliseconds 
cuda radius search took 3.039 milliseconds 
result size is 5000000 



[!] Host octree resolution: 25

pcl radius search took 452.282 milliseconds 
pcl radius search took 451.796 milliseconds 
pcl radius search took 452.030 milliseconds 
pcl radius search took 456.698 milliseconds 
pcl radius search took 478.862 milliseconds 
pcl radius search took 479.827 milliseconds 
pcl radius search took 468.884 milliseconds 
pcl radius search took 459.850 milliseconds 
pcl radius search took 454.367 milliseconds 
pcl radius search took 451.766 milliseconds 
pcl radius search took 460.931 milliseconds 
pcl radius search took 467.174 milliseconds 
pcl radius search took 454.697 milliseconds 
pcl radius search took 467.473 milliseconds 
pcl radius search took 451.788 milliseconds 
pcl radius search took 451.564 milliseconds 
pcl radius search took 452.378 milliseconds 
pcl radius search took 453.132 milliseconds 
pcl radius search took 450.900 milliseconds 
pcl radius search took 454.365 milliseconds

```




