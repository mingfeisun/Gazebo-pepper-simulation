# Gazebo_hand_model
* Author: Ziming Chen
* Created: 2018-07-27
* Updated: 2018-07-28

This model simulates a human hand in the Gazebo environment.

* [Hand Model](https://free3d.com/3d-model/freerealsichand-85561.html)

Related helpful tutorials and websites:

* [Gazebo: Make a model](http://gazebosim.org/tutorials?tut=build_model&cat=build_robot)
* [Import Meshes](http://gazebosim.org/tutorials?tut=import_mesh&cat=build_robot)
* [Gazebo学习总结之制作一个模型及导入网格](https://blog.csdn.net/xk_t9_98568/article/details/21116575)
* [UV Mapping 简书](https://www.jianshu.com/p/37af2e0cac4e)
* [使用Blender的UV映射制作一个地球](https://blog.csdn.net/g6uqwseseo/article/details/71195970)

## Final method:
- 用blender将obj网格(mesh，相当于手的一层皮肤)导入，mtl文件会自动关联，option+Z开启渲染。可以复制并调整光线，注意调整网格的初始状态，包括rotate, scale, translate等属性。
然后导出为dae文件或者obj+mtl文件，注意勾选导出的参数。然后将dae文件写入sdf模型中，添加物理属性进行仿真。不断调整模型中box的参数，调整blender中网格的状态
，使box的轮廓能包住手，并且使box重心和手的重心重合。

### Previous tried method:
将dae文件导入，在UV贴图模式下，将mapping的jpg文件加载，就能够对皮肤进行渲染。但有一个问题：不能导出为已经渲染好的dae文件。
但是，能导出为obj+mtl文件，然后再用blender载入，调整角度，大小，光线等参数，再导出为dae文件。

### Notice:
Gazebo用obj文件加载也可以，但是模型会出现放置状态扭曲的问题，即跟blender中模型的状态不一致（比如blender中平躺，gazebo中变为斜侧了），但是dae就不会出现这个问题

其实能直接在dae文件里引入jpg文件(如果FreeRealisticHand这个文件夹下的Hand.dae文件里直接导入了hand_mapNew.jpg的话，就不用这么麻烦了。）

Blender导出dae文件时会自动生成mappping的jpg，并且dae文件里会引入jpg
<library_images>
    <image id="hand_mapNew_jpg" name="hand_mapNew_jpg">
      <init_from>hand_mapNew.jpg</init_from>
    </image>
 </library_images>
