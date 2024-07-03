0.学习使用rust https://www.bilibili.com/video/BV15y421h7j7/


## day 1
1.使用bevy渲染图形  https://www.youtube.com/playlist?list=PL2wAo2qwCxGDp9fzBOTy_kpUTSwM1iWWd

2.模拟物理实现：开源库bevy XPBD的物理模拟 https://github.com/Jondolf/bevy_xpbd

## day 2
1.使用XPDB算法实现物理模拟，PDB算法解释https://www.xianlongok.site/post/d6327b48/#ref-anchor-10

2.简单的2D物理引擎 https://johanhelsing.studio/posts/physics-01-balls

3.直接copy Bevy XPBD部分源码尝试实现重力 --- 不会动 原因未知 需要详细阅读源码 **本方案暂弃置**

4.尝试模仿步骤2模式，套用XPBD公式实现仅线速度的重力

## day3

1.参考https://johanhelsing.studio/posts/physics-01-balls 中对于XPBD流程的实现

2.仅考虑重力的状况下，阅读Bevy XPBD源码内关于distance joint的部分https://github.com/Jondolf/bevy_xpbd/blob/main/src/constraints/joints/distance.rs

3.参考距离约束 https://www.xianlongok.site/post/d6327b48/#%E8%B7%9D%E7%A6%BB%E7%BA%A6%E6%9D%9F%EF%BC%88distance-constraint%EF%BC%89

3.实现三维空间物体之间的距离约束，以质心为连结点

4.距离约束将物体之间的距离限制死，但不考虑各种弹性等，与Bevy XPBD的演示Distance joint工程有差别。

## day4

1.了解三位空间中的旋转与四元数 https://krasjet.github.io/quaternion/quaternion.pdf

`v′ = cos(θ)v + (1 − cos(θ))(u · v)u + sin(θ)(u × v)`

`𝑞1𝑞2 = [𝑠𝑡 − v · u, 𝑠u + 𝑡v + v × u]` 

2.实现刚体在固定角速度下的自转

3.角速度固定状况下无法匀速旋转，带入的公式似乎有问题

## day 5

1.抄袭transform方法里的rotation，已经可以旋转

2.碍于工期，先抄袭xpbd内的distance joint 代码

## day 6

1.抄袭xpbd代码已跑通（质量参数写死）

2.模拟速度奇慢无比，重新check代码

## day 7

1.加入translation对位置的影响，速度恢复正常，但有动能衰减，不符合物理，需要调查原因

https://en.wikipedia.org/wiki/Lagrange_multiplier

2.转动似乎也有物理上的问题



加入外力

加入摩擦







​	