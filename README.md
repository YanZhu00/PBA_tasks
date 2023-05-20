# Physics-based Animation Tasks

tasks for course "Physical-based Animation" (spring semester, 2023) at The University of Tokyo

東京大学「物理ベース・アニメーション論」授業課題

## Original Repository  
- https://github.com/PBA-2023S/pba/tree/main

## Thumbnails
| 01_EulerMethod | 02_Momentum |
|:-----------------------------:|:------------------------------:|
| <img src="docs/1.gif" width="350px"> | <img src="docs/2.gif" width="350px"> |

| 03_SpatialHash| 04_KdTree |
|:-----------------------------:|:------------------------------:|
| <img src="docs/3.gif" width="350px"> | <img src="docs/4.png" width="350px"> |

## Get Started
**1. Downloading the repository:**
```bash
$ git clone --recursive https://github.com/YanZhu00/PBA_tasks.git
```

**2. Modify the top <ins>CMakeLists.txt</ins> to change tasks** 

Uncomment the task that you want to build.  
```cmake
add_subdirectory(01_ImplicitEuler)
#add_subdirectory(02_Momentum)
#add_subdirectory(03_SpatialHash)
#add_subdirectory(04_KdTree)
```
**Notice**
- Make sure that only one task is added at the same time.
- Build `"03_SpatialHash"` and `"04_KdTree"` with `Release` config.
