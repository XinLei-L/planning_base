#### Planning Base

这个项目主要是内容是：在学习路径规划项目中，积累到一些知识。作为以后在写代码的时候，能够轻松调用

项目内容有以下：

- Behavior：对于移动机器人来说，其行为是复杂的，行为与行为之间的关系也是复杂的。利用行为树的框架，就能够清楚的处理行为之间的关系。这对于代码的扩展也有很好的帮助
- Bresenham：主要用来得到两点之间的离散点。可用来判断两点之间是否有障碍物
- lifecycle_node：ROS2中的生命周期节点。用来控制节点的状态，然后根据节点状态运行不同的程序。其次，有时候并不是所有的节点都需要运行，只是在某个触发时刻才会运行。在触发之前，我们只需要将节点初始化，并待命，就能够在触发时刻立马运行。将节点设置为生命周期节点，有利于更好的使用运行资源。
- distance_field：主要用来将pgm地图加载成具有膨胀层的地图。目前还有问题，需要完善。

