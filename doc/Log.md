## 01/30/2018
###To do:
####1. 修改步长和梯度改变函数，使之下降的慢一些
###Log:
####1.  在geom里面写`mass="2"`可以更改质量。会在模拟中体现。默认密度是水的密度。似乎是分米单位。
####2. 差不多竖起来了，角速度还要调，最后比较震荡
## 01/31/2018
###Log:
####1. 安装了anaconda，开始菜单搜索anaconda prompt就是anaconda的命令行，可以在里面pip install，在anaconda python里面执行
## 02/19/2018
###Log:
####1. 前一阵子试图安装mujoco_py,一直出错，推测是环境问题，以及openai在Windows的代码没有经过测试。暂时不用python了。
####2. 将双摆模型换成了deepmind的模型，shoulder joint加力矩进行控制。每个trial结束输出一次当前控制位置角速度。
####3. 模型质量“1”。长度“1”。
###Plan
####1. 如果需要限制控制，可以在模型文件加入ctrllimit并将目标改为正负都可以。
####2. 继续调参数。
## 02/20/2018
###Log:
####1. 在模型文件中指定的elbow joint为ctrl[0]。
####2. 为了让terminal cost可以用界面上的曲线显示，设置最后5个state为terminal cost。
####3. 验证相对角度代码正常工作。
####4. 继续调参数。
## 02/22/2018
###Log:
####1. 验证了controlability，现在肘部驱动的模型在顶点是可以control的。
####2. 还在调参数，扰动系数和更新步长随trial number变化的函数重新写了一下，有的时候位置能差不多到，但是速度不行。
####3. 每到trial 28的时候后面就崩溃，不知道为啥。
## 02/25/2018
###Log:
####1. 有的时候28之后不崩溃了不知道为啥。
####2. 基本竖起来，速度运气好的时候还可以。
####3. 开始线性化。
####4. 线性化出现singularity。。。
####5. 有一个delta_u是正无穷，第83trial的第6index。
## 02/26/2018
###Plan:
####1. 把swingup过程线性化。
####2. 用lqr，由最高点起始加扰动，线性化，和由模型得到的线性化结果比较。
####3. 在swingup和balance之间切换，balance阶段不再reset，与时间无关，只与位置误差有关。
####4. 加入control limit。
###Log:
####1. 发现delta_u正负无穷的原因：
####生成的随机数一直是一样的，并且会有一个U为0，log(U)就得到了无穷。暂时强行给那个U赋值，可以正常线性化，但是精度很差。
####2. Swingup线性化完成，精度较差。后续考虑在修复随机数代码后，增加rollout数量，减小扰动来提高精度。
####3. 截止目前的代码和数据备份在data文件夹。
## 03/04/2018
###Log:
####1. 完成顶点线性化，暂时是用的顶点小扰动，测量两个0.1s的x分别为x_k和x_k+1。程序运行正常，还没和手算的线性化比较。
## 03/05/2018
###Log:
####1. 线性化结果和matlab按照dynamics算的不符合。检察矩阵有singularity。
####2. 把时间步长缩小为0.05s，两次扰动系数都取比较大，还是不行。
####3. 发现当初始点在顶点，则顶点的角度为0，修改状态目标。
####4. 计划继续增大扰动系数，使之与重力效应相当， 并修复随机数生成函数。
## 03/06/2018
###Log:
####1. 增大扰动系数效果不明显。
####2. matlab检查得生成的随机数基本满足均值为0，方差为1的高斯分布。加入根据系统时间生成的随机种子。
####3. 第二个bar的速度太大了，导致矩阵near singular，求逆也算的不准。。。
## 03/08/2018
###Log:
####1. 完成新建balance项目，下一步先做平衡的控制。
## 03/12/2018
###Plan:
####1. 开始加一个扰动，用lqr反馈算出控制，看能不能平衡。
####2. 减小顶点线性化的time_step，试图减小singularity。
####3. 重新检查dynamics算的线性化，上面角度是0。
####4. 检查balance程序。
###Log:
####1. 设定cost function的Q, R矩阵，用matlab的lqr函数算出了最优的反馈控制系数矩阵K。
####2. 并不能平衡啊啊啊啊，崩溃。。。。。不过想一想感觉time_step在线性化程序中改变之后不影响之后平衡的过程，因为结果是得到矩阵A, B。
## 03/13/2018
###Log:
####1. balance程序逻辑正常。
####2. 线性化减小time_step没用。
####3. 重做了dynamics的线性化，好像也是对的。反而lqr算出来的k更奇怪了。
## 03/14/2018
###Log:
####1. 论文的模型是由单摆衍生出来的，质量集中在棒端点，和我们模型的棒不同。重新手算动力学。
####2. lqr使用方法
## 03/20/2018
###Log:
####1. 陈总超级大佬帮忙手算了dynamics，要用质心的平动加转动才可以。
####2. matlab算出了continuous time的gain matrix K。
####3. 重新写了balance代码，新的balance代码用上面的K可以使双摆平衡。
####4. 不再用加随机控制的方法使初始位置偏离0位，而是直接在程序开始和复位时将随机值赋给模型状态参数，这样算出来的矩阵没有singularity。

> ##应该把swingup线性化的初始扰动也用这种方法进行更改。注意gear。

###Plan:
####1. 弄清discrete time和continuous time的A，B矩阵关系，用matlab求出discrete time的K应用到balance代码中。
####2. 和线性化代码求出的A，B对比。
## 03/21/2018
### Log:
#### 1. 整理了continuous time和discrete time的系统方程的矩阵转换，用matlab的lqrd命令求出离散情况的K矩阵，在mujoco中可以使双摆在顶点平衡，但是如果u的权重R设的太大（相比Q），就会在一个非0的位置平衡。所以R取小点比较好，感觉用离散的K效果差一些，稳定得慢。
#### 2. 线性化的A矩阵比较符合，B差的有点多。
#### 3. 找到问题了，真是醉了，模型文件定义motor的时候gear是2，导致实际作用的力矩是所给值的一半。
#### 4. 新问题，虽然A,B矩阵看着基本一样，然而求出的K差很多，不平衡，甚至K是正的。。。
#### 5. 角度在模型中会被转化为radian，在模型文件中设置angle也没用，不能这样减小误差了。
### Plan:
#### 1. 可以整理一下过程然后试试两组A, B的输出是不是比较吻合。主要改一下swingup线性化的初始扰动，重新找最优。
## 03/22/2018
### Log:
#### 1. 驾照考过了哈哈哈！
#### 2. 感觉高斯分布随机数生成函数不是很好，如果做10000次实验会准确一些，理论上那个求逆结果应该是对角矩阵，实际求出来有误差，所以做了svd，用分解出来的对角矩阵进行计算得到A,B然后LQR求K，这样求出来K代入mujoco差不多可以平衡，需要时间比较久，震荡较大，不过还是可以平衡在顶点的。
#### 3. 对swingup做了修改，然后又有bug了。。。
## 03/23/2018
### Log:
#### 1. 修改了初始扰动，更合理了。之前把gear改成了1，相应的改了控制扰动的值和更新步长，还是有一些不对。
## 03/24/2018
### Log:
#### 1. 线性化结果不对，有一些的G矩阵是对的有一些不对。感觉线性化数据组织应该没错，不然应该都不对，计算应该也是对的，不知道是不是rollout次数少了导致误差或者恰好处在角度修正的临界。
### Plan:
#### 1. 读取控制值进行模拟。
#### 2. 怀疑是角度修正的问题。
## 03/26/2018
### Log:
#### 1. 读取控制值完成。
### Plan:
#### 1. 将扰动减小，再次进行顶点线性化。
#### 2. 整理过程准备报告。
## 04/02/2018
### Log:
#### 1.整理了4个过程和老师讨论。下一步将所有的扰动减小，修复一下角度修正的潜在问题，控制限制使用增加penalty系数R的方法实现。加油！
## 04/04/2018
### Log:
#### 1. dlqr很敏感，lqrd就还好
#### 2. 5000个trial快进, 避免U为0可能导致结果偏向一侧，所以取0.0001。扰动系数都是0.02。结果还可以，比较稳。
### Plan:
#### 2. 不做SVD了，缩小扰动后直接读取数据到matlab计算K，如果dlqr不行就转换成连续的用lqrd。
## 04/04/2018
### Log:
#### 1. 没感觉ERA和Least Square在这里区别很大，毕竟ERA只能time invariant用，先要用Least Square求H之后SVD，然后再解A,B矩阵。

> matlab可以调用c语言函数和dll。考虑之后把mujoco的dll全部放到matlab里，然后找办法处理h文件，再把主函数文件里其他函数用c写了mex导入，主函数和计算函数可以用matlab写，实现matlab运行mujoco toobox。

#### 2. 随机数没问题，生成2000个算出来均值方差都比较稳，U不能是负的，因为log的定义域，改变大小生成的随机数最大最小值均值方差变化不大。
#### 3. 角度修正方式改为上臂根据当前角度改变目标角度，不对记录的角度数据做修正，因为据观察一般上臂不转一整圈，当然加上也可以，不过要对目标角度进行更改而不是记录的角度数据。小臂改为对目标角度进行修正。如果不更改角度数据，则delta_x就不会出现jump，线性化应该ok。而优化时cost function 中是目标和实际值的差，所cost不会因为转圈而变得非常大。
#### 4. 经过验证，目前的0.05：1的快进比，不会对结果造成影响，得到的K不论是否快进都是一样的。
### Plan:
#### 1. 更新步长和扰动偏差值需要改，先很大后很小好像并不很好。
#### 2. 调cost function参数。
## 04/08/2018
### Log:
#### 1. swingup过程几天的调试，加入了control penalty，随着更新，control penalty在cost function中的比重会变大，然而还是做不到deepmind那样抖一抖抖上去的效果，所用的control还是很大，转很多圈，但大体上能上去了。
#### 2. 用小step_size再试试。并不成功。
## 04/09/2018
### Log:
#### 1. 把swingup optimization的程序输出完善好，保存优化结果和参数。
#### 2. 用师姐说的方法使扰动和更新步长系数都非常小，更新步长小于0.1，很慢但是好像是在往正确方向优化。
## 04/11/2018
### Log:
#### 1. cost function 严格取最后一个step的，在往上面摆，但是cost的系数对局部最优点的影响很大，如果没调好就会停在局部最优。
## 04/12/2018
### Plan:
#### 1. 可以试着调一下快进系数。
#### 2. test.cpp修改测试，如果是realtime的就不能用了。
#### 3. 继续调cost参数。
### Log:
#### 1. 快进系数调到0.08:1还是可以的，0.05就不行了。
#### 2. 500个trial耗时约一小时，得到
> Control: 5.2322 6.7808 4.3786 4.4418 -1.318 0.8470 -3.392 -2.478 -5.843 -3.127 -0.131 1.0067 -2.282 -1.381 -0.354 -2.422 3.2325 1.3234 0.8044 -0.000 -4.135 3.8265 2.1453 -4.335 -15.19 -9.806 -3.155 1.1516 4.2505 10.018 
#### 3. test速度大约是目前带动画模拟的5倍。
## 04/13/2018
### Log:
#### 1. 顶点balance任务用不同模型如果起始位置不同则x_goal不同，需要注意做角度修正。修正了还是不能平衡，需要验证修正的结果对不对。
## 04/16/2018
### Plan:
#### 1. 把swingup线性化做了
#### 2. 继续调参数，让终点更接近
#### 3. 能不能用matlab的fmincon
#### 4. deadline May 18th
#### 5. 验证角度修正
## 04/18/2018
### Log:
#### 1. 角度修正验证成功，把下面起始的model放进去手动移动到平衡位置加小偏差可以稳定。
> cd C:\Users\58306\Dropbox\Double_Pendulum\simplify_swingup\x64\Debug
> simplify_swingup dm_acrobot.xml s 420000
## 04/21/2018
### Log:
#### 1. 尝试方向：更新Q，初始位置设为90度。
#### 2. 时间改为3s。
#### 3. matlab的vs2017编译器加载好了，问题在matlab2017a自身是不带vs2017的支持文件的，要从网上bugreport下载复制进去，位置："C:\Program Files\MATLAB\R2017a\bin\win64\mexopts"。加载mujoco的dll库成功但是有warning，还没测试能不能用里面函数。
## 04/22/2018
### Log:
#### 1. 更新Q效果不明显，感觉更新方式要有理论支持才行，时间就3秒吧，毕竟之前的结果比较好的实验里三秒感觉差不多刚好，只是控制需要优化的更好。
#### 2. 减小了更新的stepsize，扰动系数没变，感觉扰动系数影响不明显，现在这个还比较合适，更新步长应该设小一点，可以不随更新过程变小因为越接近最优点cost function的变化会越小。
#### 3. Cost function的系数慢慢在调，和更新步长要配合。
#### 4. 初始控制值用sin感觉比全是0效果要好一些，这个还要调。
### Plan:
#### 1. 找时间做swingup线性化
#### 2. 用matlab调用整个mujoco感觉是个浩大的工程，可以看一下加载自己写的dll之后matlab调用新的c代码，c代码调用dll函数是不是需要特殊语句，如果要的话就很麻烦应该不行。然后fmincon应该是要调用模型，而不是时时给输入输出值，所以vs调用matlab的fmincon应该是不行。
#### 3. 继续调参数。
## 04/23/2018
### Log:
#### 1. 用vs2017生成了dll和lib，注意如果要生成lib文件，头文件里的函数声明至少有一个函数前面要加上__declspec(dllexport)，表示输出函数。注意生成库文件和用库文件的两个项目的x64和x86设置要一致，如果选择x64则重新生成的相应库文件在x64文件夹而不是debug文件夹。资源文件里只添加lib文件。vs里测试库成功。
#### 2. 可以尝试将参数写在文件里，这样可以同时开多个窗口模拟。
### Plan:
#### 1. 多窗口模拟
#### 2. 继续看matlab能不能行
#### 3. normalize state，用新的Q并且改成running cost
#### 4. random initial control value看converge到的地方是不是很敏感。
## 04/23/2018
### Log:
#### 1. 参数写在文件里了。
#### 2. 因为是命令行，不能多窗口。
## 04/25/2018
### Log:
#### 1. 机器是64位的，所以dll要64位的matlab才能用。dll的源文件后缀是cpp，头文件就要加extern c的那一套，不然matlab不认。	
#### 2. 加载了外部dll之后，在matlab程序里一定要用calllib才能调用，也就是说，要把mujoco主程序passive simulation的部分用matlab重写成m文件。现在调用库遇到问题哎。
#### 3. mujoco论坛看到的和matlab的配合使用好像都是通过通信而不是直接调用。
#### 4. 真是醉了啊！直接在.c文件里include外部库的头文件，然后mex testdll2.c dll64.lib就可以了，关键在于只能.c不能.cpp。这种问题真是太无语了。
#### 5. 既然如此，只要把mujoco的库文件全部放在matlab的当前路径文件夹下，重新写一个mujoco进行模拟的c文件程序，然后mex编译成一个函数，函数名是c文件名，就可以用fmincon调用了。
#### 6. 看了一下师姐写的matlab代码，fmincon调用的是cost function的函数，那给定30个timestep的控制，在这个函数中调用mujoco，加入所有控制，进行一次完整模拟得到所有timestep的state，算出cost，返回，这样fmincon也许就可以优化出最优的control了？
#### 7. 所以暂时的结论是matlab可以的，找师姐确认一下。
## 04/26/2018
### Plan:
#### 1. 改成running cost
#### 2. normalize state，得到新state输出到文件的值。
#### 3. random initial输入，记录到文件！画cost图！
### Log:
#### 1. 改了running cost, R一直不变，Q在最后一个step切换成terminal的Q。
#### 2. 加了random initial control，记录到result.txt。
#### 3. normalization完成。开始实验
#### 4. 问过师姐，fmincon是那样用的，之后可以试着调用mujoco。
## 04/27/2018
### Log:
#### 1. 算法的作用，优化程序，更少的计算资源时间消耗。写程序之前，从大局计划需要的功能和数据结构，达到增加可读性，可复用性，规范性的目的。
#### 2. running cost效果感觉很不好。
#### 3. 函数库实验成功，在文档VS的MyProjects里。
> ## 把函数写成函数库，从大局计划需要的功能和数据结构，达到增加可读性，可复用性，规范性的目的。
#### 4. 改成running cost下降比较慢需要更多的实验次数，有的时候优化的方向对，可能需要加大扰动。
## 04/28/2018
### Plan:
#### 1. 适当加扰动，增加更新步长限制，看能不能下降更快。
#### 2. 确定较合适的Q,R值，只改初始输入，截图看是不是局部最优不同。
### Log:
#### 1. 按规范改写完成求逆函数。
## 04/29/2018
### Log:
#### 1. 加扰动效果不好，增加更新步长限制效果也不好，又把他们调的比较小，感觉没什么影响。看来关键还是在cost function。
#### 2. 改写了求逆和文件写入和参数读取函数。
## 04/30/2018
### Plan:
#### 1. 改写cost function，角度修正。
#### 2. matlab
## 05/01/2018
### Log:
#### 1. 两种方法用上matlab，第一种是把cpp改写成c的，注意c没有using namespace和默认参数之类的；第二种是把cpp文件做成lib。现在在尝试第一种。
#### 2. 调了cost参数，增加trial数到2500左右，还要观察，更新步长要小。
#### 3. c语言的头文件一定要加.h！
#### 4. #include "mex.h" 并把其他的cpp的头文件去掉，注意math.h最好用mujoco头文件里面的，不然可能是指向cmath的，const int不能识别，用define代替。时间函数改成c中的time()返回time_t型秒数，其实是long。把main函数改写成接口函数，注意mxGetPr(prhs[0])用法是取prhs[0]的内容，也就是一个指针，直接定义指针类型，不用初始化数组长度，后面可以直接按数组调用。返回值按指针传递。
> mex swcost.c mujoco150.lib
这样就可以，在matlab中调用的时候要记得给swcost参数，不然可能matlab会崩溃。
#### 5. 用了fmincon，结果不好，之前改写的matlab调用mujoco测试了，可以成功输出cost值。
## 05/02/2018
### Log:
#### 1. 用单摆做完全可以摆上去并且平衡。
#### 2. 师姐给了fmincon参数选择。
## 05/28/2018
### Log:
#### 1. 旋转逆时针为负。小车是d-qpos[0],bar是1。
> cd C:\Users\58306\Dropbox\Cartpole\swingup\x64\Debug
> swingup cartpole.xml s 420000
## 05/30/2018
### Log:
#### 1. 考虑给顶点线性化加角度补偿，这样所有过程只需要一个模型。
#### 2. cartpole一根bar的成功完成swingup和balance。
#### 3. 现在只需要一个模型就可以了，其实做balance控制是用角度差，只要线性化的位置是对的，也使用角度差就可以了。
## 06/05/2018
### Log:
#### 1. qpos[1]是高度。站起来目标大于-0.2。
> cd C:\Users\58306\Dropbox\Hopper\standup\x64\Debug
> standup hopper.xml s 420000
## 06/06/2018
### Log:
#### 1. result显示成功但是display的结果不对。
## 06/12/2018
### Log:
#### 1. 大量时间消耗在求逆的那一步。
## 06/15/2018
### Log:
#### 1. 试下不把站起来后控制清零，高于一定值cost设为0
#### 2. 把stochastic的方法改成分别给每个控制量加扰动。
## 06/16/2018
### Log:
#### 1. step_coef需要变吗？单通道情况最好不变，stochastic方法可以变可以不变看模型。
#### 2. drop和不drop哪个好？不咋影响。
#### 3. 单通道方法如果给的扰动太小了，cost变化很小，不会有梯度下降效果。
## 06/17/2018
### Log:
#### 1. 需要在进入模拟循环前加入t_init = d->time同步时间，不然第一组有偏差。
#### 2. 至于每一次相同模拟结果不同，position数值的误差是3e-4的数量级，应该是正常的数值误差吧，出现的位置非常固定，没发现随任何参数变，应该是因为程序不存在随机性了。
## 06/25/2018
### Log:
#### 1. 单通道方法的问题对cost影响比较大，随机方法这方面就好很多，还是用随机方法。
#### 2. 用openmp并行执行效果不好，调度耗时比循环时间还长，真正需要并行的求逆部分加了并行之后报错，内存溢出，可能是determinant函数和transpose用的地方不一样，有一些没发现的嵌套不能并行。感觉还是可以实现的。
#### 3. go语言自动支持并行和并发，可以用cgo方法调用c的库，但是优化更新的每一步是在上一步的基础上，整体是个串行的，求逆部分可以并行，应该还是可以减少一些时间消耗。
#### 4. hprc的计算机需要用命令行。
#### 5. hopper注意gear！！！
#### 6. gear越大真正施加的力越大！
## 06/27/2018
### Log:
#### 1. gear设置为1还是不行，暂时不管gear，觉得线性化不成功是foot和floor的撞击导致的。
#### 2. 先试试把delta_x的式子改成x的。有singularity。
#### 3. 然后pin住torso，看能不能平衡。
#### 4. 现在是没管x方向，可以问下老师这样对不，我感觉是对的。
#### 5. 好像也可以试下用optimize方法把平衡当成optimize的task，站的凑合现在，如果不把所有关节都限制也许效果会更好。
## 06/28/2018
### Log:
#### 1. meet的结果是由于hopper的脚板和地面碰撞时倾斜方向不一样导致整体模型一直在变，这里要把模型当成黑箱，所以只有输入和observable的输出的信息是可以用的。
#### 2. 开始做鱼。
#### 3. 用optimization平衡的话，是用的delta的式子，此时减去的x_nominal式子后面要加u_nominal这样左边才能用一样的x_nominal减，求u_nominal的方法是给一个step，优化cost是位置差，就没有time sequence了。
> 4. 把函数入口点从wWinMain改回main就可以在命令行直接load model。此时会有mjkey的error，但不影响运行，如果把mjkey.txt放到.vcxproj也就是项目文件相同目录，则error消失。命令行启动和双击exe启动需要exe相同目录有密钥文件和model文件；编译启动需要vcxproj文件相同目录有密钥文件。main是适合console的，wWinMain适合GUI。main的话编译的时候会试图运行，如果vcxproj文件相同目录没有密钥文件，则会弹框报错。解决方法是在读密钥之前让程序自己退出。方法是利用编译启动没有传入参数，设个判断然后finish()。
## 06/29/2018
### Log:
#### 1. 鱼的模型因为上面general的里面有对ctrl_limit的定义所以下面actuator一定要有ctrl_limit，先试试看吧。
#### 2. qpos0，qpos1是平行floor的两个方向，qpos2是垂直floor方向，qpos4俯仰，qpos5滚转，好像不对，应该是qpos3，4，5，6是quaternions。
> q = (w, x, y, z). Here (x, y, z) is the rotation axis unit vector scaled by sin(a/2), where a is the rotation angle in radians, and w = cos(a/2). 
#### 3. sensor是传感器，可以读到需要的数据，鱼的actuator是position servo，kp是pid里面的位置增益。general里面的solref什么的是为了模拟流体的力。
#### 4. print result程序的位置必须在最后，否则print出来的是初始的值！！！！！！！！！！！！！！！！！！！！！！！！
## 07/01/2018
### Log:
#### 1. d->xmat[17]是torso的orientation matrix第九个元素？也就是z轴投影？如果真是这样这个设计也够。。的，然后这些数据不需要sensor就可以获得。
#### 2. 666好像真是这样的23333，所以d->geom_xpos[11]-d->geom_xpos[5]是z方向距离，d->geom_xpos[10]-d->geom_xpos[4]是y方向距离，d->geom_xpos[9]-d->geom_xpos[3]是x方向距离。
## 07/02/2018
### Log:
#### 1. 可以把优化的程序加上前后阶段衔接。
## 07/06/2018
### Log:
#### 1. 把矩阵求逆改成迭代更新，速度显著变快，大约快了4倍多，鱼的翻转和游到目标的任务效果都不错。接下来做swimmer，并且再把swim做的准确一些。
## 07/08/2018
### Log:
#### 1. d->geom_xpos[6]和d->geom_xpos[7]大约是的，只有x，y方向运动，目标的x，y坐标都是1。 
## 07/10/2018
### Log:
#### 1. for循环终止测试格式错误的解决方法是，把for循环初值终值都用数字明确表示而不是用变量：#pragma omp parallel for \n for (int q = 0; q < 60; q++){}。
## 07/12/2018
### Log:
#### 1. 6节的swimmer可以游进目标了，但是开始一段是倒着游的，和deepmind的solution不太一样，模拟耗时8小时。
#### 2. 试了一下hopper，1小时模拟了1200000秒，cost曲线在最后还处在下降的趋势，需要更多的时间。
#### 3. 函数库基本写完了，下一步改主函数文件。
#### 4. acrobot的costfunction按照deepmind的改了一下，用d->site_xpos[3]-d->site_xpos[0]的平方加d->site_xpos[5]-d->site_xpos[2]的平方，表示端点到目标的距离。
#### 5. hopper的costfunction可以改一下。
## 07/13/2018
### Log:
#### 1. fish把z轴的系数变成一半，能游进目标了，感觉很奇怪，如果调大系数会偏高，调太小会偏低。
#### 2. 双摆还是不行，比其他的更noisy，perturbation和更新的系数需要调，但注意别弄成每个trial都改变，感觉后面还是perturb不够找不到解，可以试试把时间变长感觉可能有用。
#### 3. hopper的cost需要按照deepmind的改一下试试。
## 07/16/2018
### Log:
#### 1. hopper的d->xpos[5]是torso的z坐标，d->xpos[17]是foot的z坐标，d->subtree_linvel[0]应该是torso质心的x方向速度了。
#### 2. hopper的cost function按照deepmind的改了，在设置控制量那里加了判断减少循环次数。
## 07/17/2018
### Log:
#### 1. d->geom_xpos[18],d->geom_xpos[19]是小球的x，y坐标。
#### 2. d->geom_xpos[27],d->geom_xpos[28]是指尖的x，y坐标。
## 07/19/2018
### Log:
#### 1. d->subtree_linvel[0]应该是cheetah的torso的x方向速度。
#### 2. 把每次更新的最大值改小到0.04，更精细一些，找到了向前跑的解。
## 08/01/2018
### Log:
#### 1. 重做单摆，d->xmat[17]是pole的zz方向orientation，前一个矩阵应该是惯性坐标系。
#### 2. 单摆swingup可以做到控制值小于1，和deepmind要求一样，用时8秒。
## 08/03/2018
### Log:
#### 1. 修改了matlab调用mujoco的程序，改成每一步执行，输入参数是控制量，初始状态，时间步长，输出是一步后的状态。
## 08/05/2018
### Log:
#### 1. 线性化大体上写完了，程序参数减少为程序名和模型名，跑完了就结束，不再定时间。
#### 2. 用的迭代的方法，如果测试成功，变量还可以精简，现在有一些变量是按照求逆的需求设置的，所以还是会存很多东西，但是不求逆就用不上。
#### 3. 起来了测试一下。
#### 4. 取消了做线性化的时候对控制值的限制，control nominal还是要变换。
#### 5. matlab结果和c的state nominal对不上。
#### 6. reset之类的就不跳转到-0.2再执行了，这样可以立刻通过mj_step设置值，更准确！！！！
## 08/06/2018
### Log:
#### 1. 改成每个step单独设随机值计算，还是不行，和之前一样，尤其是速度算的特别不对，起来了看下delta x1和x2的值是不是和计算的符合。
## 08/07/2018
### Log:
#### 1. bug猛如虎啊，居然time-t_init<0.1这个判断都会莫名其妙在有一些step出问题，可能因为数据中存的0.1其实比0.1稍微大了一点点，然后就呵呵了，改成0.099暂时OK，c和matlab的模拟结果一致了。
## 08/10/2018
### Log:
#### 1. 修改了mj_step()的位置，使模拟进行的更加合理，消除了不应该有的step，通过上一个bug增加理解，还可以吧，然后控制赋值不再放在for循环里，可能稍微会快一点，程序进一步精简。
#### 2. 修改了matlab调用mujoco的c程序，后面finish()里面要加上参数m和d，不然不能释放空间，跑十几次后会崩溃。
#### 3. 经过师姐的lqr程序检验，least square方法求的A,B可以用，在维度较少的情况下矩阵求逆比迭代耗时少而且更精确，两种方法都在里面而且切换方便。
#### 4. 修改openloop和display的程序，避开上面那个猛如虎的bug。
#### 5. +=后面的加法要加括号。
## 08/11/2018
### Log:
#### 1. 更新matlab程序对result和linearization的读写，尽量保留原格式的读写。
## 08/13/2018
### Log:
#### 1. 尽量使各个文件结果输出可以直接用到后一步的读取。
#### 2. LQR那一步尽量使参数能在matlab最开始的地方更改，特别注意mexstep.c文件里要先load model才能用m->nu这些参数，读取输入参数指针那里先后顺序很重要不然matlab会崩。
#### 3. mxGetPr只能用于取得实数数列，字符串要先用mxCalloc分配内存然后用mxGetString把prhs输入的字符串赋值给char数组。
#### 4. matlab里面字符串要以变量形式传入才可以，比如先x='string'，x作为调用c函数的输入变量。
## 08/16/2018
### Log:
#### 1. 双屏，关键是
> glfwMakeContextCurrent(window);
> glfwSwapInterval(1);
需要在所有对每个window不同操作的前面放这两句，相当于把数据流指向需要的window。
#### 2. 双屏完成，控制也可以分别设，从render开始，simulation，setcontrol函数都要分开写，尽量保证不干涉吧，m，d，m1，d1要分好，就是显示的位置不太好。
#### 3. glfwSetWindowPos()可以在定义window后把window位置挪到指定坐标，其实是个简单的问题，中文搜半天都没有，英文一下就有了，真是浪费时间。
#### 4. 程序最后打包的时候一定注意每个的初始状态设置！！！！！！！！！
## 08/20/2018
### Log:
#### 1. 读取control nominal的程序一定要在模拟之前！！！
#### 2. 现在双屏可以分别同时显示加噪音不加控制和加噪音加闭环控制的结果，闭环控制就是下一步控制等于当前-K*delta_x加噪音加下一步最优控制量。
#### 3. swimmer的sysid增加rollout数量，因为state数量变大之后总参数量是平方增长，所以需要很多很多的rollouts。
## 08/23/2018
### Log:
#### 1. 在重新生成扰动进行线性化结果检验时，要注意如果在setcontrol里面写while循环模拟整个轨迹，作为setcontrol参数的time是不会变的，只有mj_step之后通过重新调用传参更新，所以循环中时间处理应该是用d->time。
#### 2. 将一个step的states当成一个矢量计算偏差。
#### 3. 检查收敛效果缩短rollout数量，有的地方曲线更加抖，所以会更敏感，效果会差，正常。
## 08/27/2018
### Log:
#### 1. 鱼的matlab代码改写，后面再弄下display，如果不成功或者耗时过长可以试下upright，因为这个actuator不一样不知道会不会有问题。
#### 2. 把读入c的TK输出看看，感觉c的结果是TK不对，但是matlab里面如果噪声加上了TK应该是对的。
#### 3. 总结前两个例子的结果。
## 08/28/2018
### Log:
#### 1. 鱼的m->nq是7，因为free joint描述旋转用的quaternion，有4个参数。
#### 2. 模型对控制的限制一定要搞清楚！！！！！！
#### 3. TK读的是对的。
#### 4. 重做swimmer6的sysid。
## 08/30/2018
### Log:
#### 1. 6-link swimmer在matlab里和mujoco里都可以实现整个过程但是区别不大而且mujoco里面不是每次都能成。
#### 2. 发现有的TK做display视角会出bug。
#### 3. 总结+输出看是不是真的有区别+鱼的display。
#### 4. TK中数据如果位数过多会产生视角的bug，matlab的reshape出错点运行跑的是其他main函数，重启一下可能ok。
## 08/31/2018
### Log:
#### 1. linearization误差分析比较+iteration。
## 09/09/2018
### Log:
#### 1. 以后都去掉所有的control limit，重做了swimmer6。
#### 2. 算信噪比，cheetah之前也是加了limit，需要重做。
## 09/10/2018
### Log:
#### 1. cheetah重做之后效果比之前好很多，这种重做过部分的项目文件夹下面会有很多之前的老文件，最后整理记得删。
#### 2. 把spike出现的几个step重做，看效果 + 调整swimmer方向。
#### 3. cheetah的线性化结果非常不好，但收敛很快。
## 09/12/2018
### Log:
#### 1. 画convergence图确定收敛所需rollout数量。
#### 2. 没有急转弯的sysid。
#### 3. 把中间的大spike区噪音去掉。
## 09/14/2018
### Log:
#### 1. 准备视频+15linkswimmer。
#### 2. 和你在一起，真幸福！
#### 3. 把dx，du数据输出，再单独perturb状态和控制，看数据的scale差的多不多，再做更大噪声的sysid看是不是在那个噪声level才work。
#### 4. 之前sysid的时候，噪声太小导致dx，du过于小，线性化结果会被round error影响很多。
#### 5. 老范我爱你！
#### 6. check data + 15linkswimmer，下周五见面。
## 09/15/2018
### Log:
#### 1. 15linkswimmer还得重做。
#### 2. check data+是不是固定噪声level才work+15linkswimmer+急转弯放入视频+鱼重做。
## 09/16/2018
### Log:
#### 1. 不是固定噪声level。
#### 2. check data+15linkswimmer+急转弯放入视频+鱼重做。
## 09/17/2018
### Log:
#### 【来自宝宝的啾啾 -3-】
#### 1. 急转弯的如果扰动加到能看出不同的程度，线性化结果就很不好。
## 09/20/2018
### Log:
#### 1. 在做python作业，在命令行用pip install jupyter和ipython，然后命令行在ipynb文件目录运行命令jupyter notebook。
#### 2. cygwin的这个仿linux的界面，目录的slash和windows命令行是反的，应该是/否则不识别。
#### 3. 安装完wget之后添加环境变量和path，要重启cygwin才能生效。
## 09/26/2018
### Log:
#### 1. 鱼的开环有了，闭环要考虑quaternion的四个变量不是都独立的。
#### 2. 15的太耗时了。
#### 3. 画了data的图。
## 09/27/2018
### Log:
#### 1. 没有急转弯的那个，改角度看有没有变化。
#### 2. 模拟，做平均，看开环和闭环谁的表现更好，确定这个的闭环是ok的。
#### 3. 在0.2用0.1结果的成功率和用0.2结果的成功率。
#### 4. AB矩阵在不同noise下是否有区别。
## 10/02/2018
### Log:
#### 1. 想让变量出现在工作区，不能加function。
#### 2. 可能是扰动对状态太大，因为是弧度制，要乘60，而对控制和xy位置不是。
## 10/03/2018
### Log:
#### 1. 猜测那个qacc无限大的warning可能是对角度加扰动因为是弧度制加的过大超过限制什么的导致的。
## 10/13/2018
### Log:
#### 1. 用高维cartpole模型验证闭环部分是ok的。
#### 2. 问下rrd，做双摆。
## 10/21/2018
### Log:
#### 1. matlab结果和c的display不一样，重写lqr。
#### 2. 三窗口。
#### 3. 双摆加油！
## 10/25/2018
### Log:
#### 1. cart2pole之类的，放到hprc上
## 10/30/2018
### Log:
#### 1. 用notepad++的比较功能有可能会把函数复制的乱七八糟，注意大括号位置。
## 11/2/2018
### Log:
#### 1. 双摆闭环+另外几个的误差图。
#### 2. 鱼。
## 11/7/2018
### Log:
#### 1. 鱼不行，把range之类的限制去掉了也不行。
#### 2. mex 用octave可以，命令行没有了清空命令行窗口。
#### 3. 给qpos3456赋值的时候，必须四个都赋值，把6算出来，不然345的值也会和赋值的不一样，出现错误的原因是4初始值是1，5目标值是1，如果扰动使它大于1，就不对了。
#### 4. fish文件夹的mexstep.c是鱼可以用的，包括其他matlab代码。
#### 5. swimmer模型range去掉看有没有变化，看是不是把控制限制真的去掉了。
#### 6. solver parameter模拟流体的参数。
## 11/8/2018
### Log:
#### 1. swimmer的流体模拟，应该是阻力与速度和位置超过限制的部分有关，超的越多力越大，但是没有saturation，不知道这样会不会造成问题。
#### 2. 鱼怎么处理四元数啊23333。
#### 3. 再弄一下acrobot参数。
## 11/10/2018
### Log:
#### 1. 为什么双摆gradient这么大。
## 11/12/2018
### Log:
#### 1. 鱼的四元数，给三个加扰动，有大于1或小于负1的，减2或加2，因为旋转是连续的，-180度也是+180度。
#### 2. 角速度的状态值会在一个非常大的范围变化，但给的ptb是一个量级，有的就不准，在状态值特别大特别小的地方误差比较大。
## 11/14/2018
### Log:
#### 1. 尝试iteration方法，0.1ptb线性化误差很大，0.02ptb误差也有点大，相比least square没有明显改善，0.01ptb的稍微似乎好一点点。
#### 2. iteration感觉还不如ls，再多做几组比较一下。
## 11/17/2018
### Log:
#### 1. 确定系统辨识ptb大小的时候要和max u做比较看百分比决定range。
## 11/26/2018
### Log:
#### 1. 3-link swimmer做成ppt了，sysid大约噪声和umax比在0.2，0.3之间error最小，timestep越小越好，闭环噪声越小越好，太大了误差会突然增高。
#### 2. 6-link swimmer规律一致，做ppt。
#### 3. 考虑重做鱼和acrobot试一下fmincon。
## 12/03/2018
### Log:
#### 1. 鱼四元数，加了噪声平方和总是会超过1。。。
#### 2. 之后把四元数转成欧拉角再试吧。
## 01/03/2019
### Log:
#### 1. swingup里面最后把运行顺序改成sysid里那样，对比一下更改前后速度，记得备份
#### 2. display，sysid里面加local开关
#### 3. 库里的initial函数按照swingup里面的改
#### 4. 0.006s3的showcurve.py是最新的
## 01/08/2019
### Log:
#### 1. 可以加一个师姐matlab代码图形结果的功能，把平均轨迹画出来。
#### 2. 优化和精简还得做。
## 01/12/2019
### Log:
#### 1. 做优化的角度可以是绝对值，避免角度处理。
## 01/17/2019
### Log:
#### 1. cost curve如果有震荡，应该是stepsize太大了，如果出现小尖峰可能是扰动太大了。
## 01/28/2019
### Log:
#### 1. swingup和display算cost一样了，初始位置不算，最后一步算。
## 02/04/2019
### Log:
#### 1. for循环模拟和if判断的结果是一样的，可以用。
## 02/25/2019
### Log:
#### 1. fishqvel分别是torso的xyz方向速度，xyz方向旋转速度（以鱼身为坐标系），各个joint的角速度。qpos是torso的xyz坐标，quaternions，各个joint角度。
## 03/05/2019
### Log:
#### 1. text.usetex设为true后出错的解决办法在https://zhuanlan.zhihu.com/p/45203018，一定要找到net installer先下载再安装完整版。
## 03/21/2019
### Log:
#### 1. d->qpos[0-6,9]，d->qvel[0-5,8]是unactuated。
## 03/24/2019
### Log:
#### 1. vsc编译c++工程，可以用https://www.zhihu.com/question/30315894的方法配置编译器调试器什么的然后新建cpp文件写程序进去，也可以再vsc里用c/c++ project generator插件的功能create c++ project，这样就不需要自己设置json文件。两种方法都可以，但是调试器比如gdb.exe的位置要求没有空格，调试器地址，输出文件地址，exe地址要明确指定。
## 03/25/2019
### Log:
#### 1. roll角度好像有singularity！
## 04/03/2019
### Log:
#### 1. fish的sensor好像可以利用下，看下position actuator参数的影响。
#### 2. sensor的gyro是测的角速度。
## 04/05/2019
### Log:
#### 1. position actuator 和 partial feedback不成功，motor和fullstate 成功，效果还可以， euler和quaternion都成功。
## 05/28/2019
### Log:
#### 1. fish改成motor actuator并且取消tendon的constraint闭环work，timestep 0.02s差不多刚好不unstable，想要不穿刺鱼身，改solver parameters的solimp和solref没用，需要加damping，constained改成predefined，加joint range limit，timestep要小一点，0.01s差不多可以，需要10s的horizon，training时间更长。
#### 2. hopper的cost从大到小变，consider 头脚速度和头高度，增大脚，头，nose的mass，使初始控制为零时蹲下而不倒。
#### 3. humanoid初始高度不能太小，不然会和floor碰撞弹飞。
## 06/18/2019
### Log:
#### 1. cheetah找到加了control limit之后能稳定training的最大timestep，优化1s需要1min，timestep长需要优化的参数少，但收敛所需迭代次数增多，所以也不能一味增加timestep。
#### 2. 下一个project是tensegrity。
#### 3. 把openloop的程序更新到mujoco200版本，增加了并行设计，暂时没啥用，速度比老版慢8%左右，不知道是不是因为改成了计step数而不是计时间。
#### 4. 开始使用之前写的函数库，现在的使用方法是添加库的头文件和源文件，然后在优化程序中直接使用里面的函数，测试使用随机数函数成功，之后稳定了再改用dll。
## 06/22/2019
### Log:
#### 1. 鱼用0.005s，3000steps效果也不错，时间更少，所以改成这个。
#### 2. 论文改成ieee transaction  journal 的格式， 主要是去ieee官网下载template，然后替换.cls文件， 在bibliography.tex,main.tex里相应地方改成ieeetran， 有一些格式的细节，比如proof的环境，ref里是不是abrv的，要不要dash repeated names， 这些要看template里面的指导文件， 有详细说每种论文元素应该用什么样的格式和环境。
## 06/25/2019
### Log:
#### 1. 需要找到每个simulate和window的关联代码，暂时打算开3个thread，记得用库。
## 07/02/2019
### Log:
#### 1. 三个窗口可以运行了，数据正常，关键在于makescene的scn变量和makecontext的con变量每个窗口需要一个对应的，然后在loadmodel函数里每个makecontext前面需要一个makecontextcurrent切换window,这里会覆盖，只有最后一个会显示。
#### 2. 左右ui还不太正常。
#### 3. 代码上传了github。
## 07/05/2019
### Log:
#### 1.
太难了，整整改了一周多，与window相关的操作顺序很重要，ui只保留nominal窗口的，那这个窗口在各种处理中要是最后一个，init里面好像不用，但loadmodel，prepare和主循环里都需要是正确的顺序，否则ui各种错乱。
#### 2. 按暂停之后时间不一样，control也乱掉，每按一次时间差多一个timestep，是因为simulate里处理out of sync情况有个mjstep，注释掉就好了。
#### 3. info和label可以三个都有，数据也分别是各自的。
#### 4. 自带的fullscreen功能安排好了三个窗口的位置大小，可是实际使用发现callback失效，所以还是直接点击nominal的右上角最大化按钮，然后点其他窗口就又放大了又有callback。放大后旋转点击有生效区域限制，懒得再折腾了，能用就这样吧。
#### 5. mj_kinematics and mj_forward do not advance the simulation. They compute kinematic and dynamic quantities for the current timestep, and the controls do not affect the positions within a single timestep. You have to integrate forward in time, using mj_step.所以reset之后接一个mj_forward是可以的，只是update当前step的，而不会simulate。
#### 6. https://blog.csdn.net/m0_37876745/article/details/78172846 这个是解决无法打开程序数据库，要删除vc140.pdb错误的教程。
## 07/06/2019
### Log:
#### 1. test2d pendulum.xml performance_test，或者是policy_compare,model_test，只加文件名就是出动画,test2d是第一个参数，第二个一定是模型文件名，第三个是模型类别或者功能，模型类别是pendulum，dbar，swimmer3这样的，功能就是上面model_test那几个，如果既有模型类别也有功能，先写模型类别，第四个参数是功能。
test2d dbarmotor.xml dbar modeltest 0.5
改成test2d 模型文件名，模型名，功能名，noise level，除了modeltest其他功能最后noise level可以不写，不写默认为0.模型名，功能名都可以被跳过。
#### 2. 做成dll别人不好改代码了，还是别了，省点事。
## 07/07/2019
### Log:
#### 1. 外部变量在main里写extern声明但不定义（初始化），在lib.cpp里不加extern再声明一遍，可以初始化。如果是用来声明数组的size的常量需要在两个文件都初始化，但只有main里的会起作用。
#### 2. 凉了555maxstep不能设大，否则training不能跑，直接在train前面退出了。。。感觉是d[64]这个太坑了，每个d都是一个大的数据结构，64个就很多，所以如果需要改就把64改小吧。并不是这个问题。。。。。其实是大数组不能在函数里面定义，必须定义成全局。
#### 3. openloop cheetah.xml 100， 100是迭代数也就是gradient更新的次数，如果文件名不是模型名，后面需要加模型名，后面还可以加thread数，不加默认1个thread，现在多个thread就更之前开多个cmd窗口一起运行check reproducibility一样，再后面是profile开关。openloop.cpp开头的宏定义TRAINING_NUM表示每个thread连续运行多少次training，这些training的cost会存在同一个文件里，多个thread就会有多个这样的文件。但是control value存的都是每个thread最后一次training得到的结果。
#### 4. sysid2d pendulum.xml 100
100是迭代数，后面还可以加thread数，不加默认1个thread，多线程这里的iteration是总数，每个rollout互相之间是并行。
sysid2d finger.xml 0.005 20 finger 8
sysid2d 模型文件名，noise level，rollout数，模型名，线程数，最后两个可以不写，改成求逆的方法就没有iteration和convergence了。
## 07/10/2019
### Log:
#### 1. 多线程在thread function外面用srand(时间)设置种子然而线程里面产生的随机数每次都是一样的。解决方法如果只开一个线程，就在线程里面设置随机数种子，如果开多个线程，就每个线程里面用精确到微秒的时间加threadID设置随机数种子。暂时只用了第一种在openloop里。
#### 2. 数组太大内存不够的问题，可以动态分配内存给数组或者矩阵的指针，这样就没有不是const不能用来指定矩阵size的问题了，但是但是但是但是，要把stepnum这些当成参数传给函数，在函数里分配内存，再传指针出来。
## 07/14/2019
### Log:
#### 1. resetdata特别耗时间。
## 07/16/2019
### Log:
#### 1. qpos和qvel是对齐的，set 全套qpos constraint就不满足，会开，而且和string有耦合。
#### 2. tendon直接模拟的spring是spring不是string，受拉压都有力。
#### 3. tendon可以加motor，通过加相应的力模拟tensegrity改原长，通过改变zero order hold，还可以模拟motor dynamics。或者直接当前长度乘force density得到应该加的力。用motor避免state确定位置之后tendon长度也需要相应改变导致模糊，也不知道这样是不是对的，觉得不太放心。
#### 4. state要用minimal coordinates，dbar就是两个角度和两个角速度。mujoco长度单位是米。
## 07/19/2019
### Log:
#### 1. tendon加motor成功，虽然加噪声之后闭环也进不去target，效果没有很好，但是比较图是闭环比开环好。
#### 2. damping和viscosity大的话，噪声和feedback的影响会变小，会让闭环和开环最后的结果差不多，虽然还是闭环好一些，但是这两个应该在sysid的时候被model到AB中，不确定是为什么会这样。把它们取小或者置零效果会变好，取0的话openloop曲线spike会大。
#### 3. sysid的perturbation取0.05到0.0002好像效果都差不多，就是iteration取多点就行。
## 07/24/2019
### Log:
#### 1. finger的base应该固定。可以加constraint使水平bar和竖直bar一直垂直。
#### 2. 矩阵求逆，直接用的eigen库里的函数，需要#include "Eigen/LU"，然后就可以用了，定义矩阵，初始化，索引的例子都在functest里面，索引从0开始。MatrixXd b(2,2);或者Matrix<double, 5, 3>。用<<初始化必须指定全部值。
#### 3. 考虑并行就把step分给不同thread。
## 07/25/2019
### Log:
#### 1. 并行把step分给不同thread，为了并行，还是存一下每个step的AB到一个矩阵里，然后最后一起输出到文件。
#### 2. 效果太好了，虽然每一步平均时间增加了，但是需要的rollout数量急剧下降，暂时试过的两个tensegrity例子所需rollout数等于state数加control数，就是能够求出线性方程唯一解要求的最小equation数，求逆还是比当成单位阵准确的多，准确率提高了10倍。如果放在training里面，dimension大的模型可能求逆花的时间会更多，可能效果会下降，但是应该还是比现在的好。可以试一下把计算gradient的步骤用这个代替。
#### 3. sysid的noise level好像小于一个值之后再变小效果不明显。
#### 3. test和sysid试一下加noise level到输入参数。
#### 5. test一下多线程是不是work。
## 07/26/2019
### Log:
#### 1. test，sysid加入了noise level的参数，都work，sysid多线程work，时间有缩短，4线程以内差不多是按照倍数缩短，超过4线程仍然缩短但不是倍数，improvement更不明显。
#### 2. finger的policy compare在noise level 70%左右开始，就会有一定几率在一些点出现大variance然后后面又变小。但每次出现的地方不是一样的，有点奇怪，可能是因为突然产生了几个特别大的随机数吧。
#### 3. time+id作为随机数种子在sysid里好像可以，每个thread的随机数是不一样的。
#### 4. 应该不需要每次设加速度为0，因为加速度可以从受力求得，而受力是每次给的。
#### 5. eigen的matrix声明时不能只用一个参数，如果只有一行或一列，要么显式把行或列设为1，要么用RowVectorXd这样的向量，不然会有一个you are tring to call 什么的错误，双击导到resize。
## 07/28/2019
### Log:
#### 1. openloop算gradient直接改求逆不行，因为gradient的元素数是step数乘actuator数，太大，求逆耗时过长。可以考虑加momentum。
## 07/29/2019
### Log:
#### 1. int 强制类型转换是向下取整。
#### 2. training num宏好像不需要了。
## 07/30/2019
### Log:
#### 1. 好好看看前面写的有没有漏的。
#### 2. 改openloop到之前的方法，加减delta好像没什么用，policy还是一样的，没有更好，时间反而更多。
#### 3. momentum的classical和nesterov方法都没什么明显的改进，有影响也能通过调stepsize达到相同的效果。
#### 4. testnum宏可以保留，cost文件输出覆盖可以加filemode解决。
#### 5. 有空测一下用eigen库和数组哪个快。。。。。数组快！
## 07/31/2019
### Log:
#### 1. finger每iteration用20rollouts就可以， 大约3秒training好。arm也是的，用20 rollouts就可以。
#### 2. 需要加constraint才效果好一点，保持垂直。但是轨迹不是卷上去的。
## 08/01/2019
### Log:
#### 1. mju_strncpy后面长度设的不对，把后面QT覆盖了2333. 改了之后cost还是有点差别。
#### 2. 尽量避免用eigen矩阵库，赋值比数组慢2到3倍，向量点乘比数组和mju_dot慢约50倍。
#### 3. inverse相比sequential，平均每个iteration耗时更长，但可以使所需iteration数减少很多，所以总的时间减少了很多。精度提高主要是因为加delta减delta，不过这样时间会变长，因为需要两个rollouts做一次计算。
## 08/06/2019
### Log:
#### 1. 用角度做轨迹track得到的解不对，会牺牲前面几个joint使后面的达标。
27   29，36   38，45   47，54   56，63   65，72   74， 81   83，90   92，93   95
## 08/10/2019
### Log:
#### 1. 老师说不应该去要求系统给我一个motion而应该接受系统能给出的motion，确实在优化中可以通过cost function设目标，但是训练得到的motion不一定是想要的，而这个motion是由系统的特性决定，不应该强行要求一个系统优化得到不适合这个系统的解。所以这个arm的模型就不要做鱼尾的摆动了，reach target point也挺好的。
#### 2. 把arm变成swimmer，mass要大一点，不然容易不稳定。
#### 3. 加了constraint后要特别仔细注意control是不是还都有效以及state数量变化。
#### 4. density小阻力小，产生的推力也小，site的dimension不与介质产生力，但geom的size影响与介质作用的力。
## 08/13/2019
### Log:
#### 1. terminal cost要大，不然后期游很慢，很难进入target。
#### 2. bar长的产生推力大，更容易向目标游，training更快，但是受noise影响大，闭环差一些，不知道为啥。
#### 3. damping太大很难游，太小training不稳定。
#### 4. 只用50 rollouts也可以，就是training的spikes多点，很节省时间而且cost下降差不多。
#### 5. bar长和角度constraint要配合着设，没有角度constraint容易卷起来。
## 08/14/2019
### Log:
#### 1. 太难训练了，0.01闭环只能到5% noise，改成0.006，闭环data和图太奇怪了，怎么会这么小，变化太小了。。。
#### 2. sysid如果statenum之类的太大就会不出结果，是因为sysidcheck函数开头三个大数组定义，所以只能不用这个函数了。。。
#### 3. incremental cost比较大更难从头不变参数训练到最后，后面下降非常慢。
## 08/21/2019
### Log:
#### 1. 出现unknown warning type可能是max state number设的小于state数或者control数了。
## 08/23/2019
### Log:
#### 1. 像dbar这种有constraint模拟joint的模型，state之间不独立，所以只取了其中独立的放在state vector里面，这样的话每次开始一个rollout初始化的时候，需要resetdata并且forward，不然会出现模拟很不对，算出来cost特别大的情况，即使没noise cost也一直在变。
## 08/24/2019
### Log:
#### 1. velocity的penalty大了cost会不稳定，不下降，所以最后速度还是挺快的。
## 08/25/2019
### Log:
#### 1. matlab用mex调用c，好像不能强制类型转换，之前的.mexwin64要删掉，然后先mex -setup连接c编译器，后面直接mex mexstep.c mujoco200.lib mujoco200nogl.lib然后运行，这里所有lib都要写上，每次改程序都要重新mex编译！！！！！！！。
#### 2. 去掉damping之后不稳定可以增加质量。
#### 3. 出现Failed to write the updated manifest to the resource of file错误，多mex几次就会有成功的。
## 08/26/2019
### Log:
#### 1. sensor应该用framelinvel，这样速度是global coordinate的，如果用velocimeter，就是在local frame下的，速度计的三轴和frame固定，不是global。
#### 2. mjstep之后居然要手动mjforward才会更新sitexpos和sensordata。
#### 3. 那几个dependent的states如果设置值结果会不一样，其实这个不是很准，不知道咋回事，误差大约0.001。
#### 4. matlab读写文件如果用c读取，格式设置%f会准确一点，不知道是不是因为科学计数法c不能直接%f读取。
## 08/27/2019
### Log:
#### 1. matlab调用c，mex function 外面的全局变量在仍然是全局的，所以load model只需要第一次。
## 08/28/2019
### Log:
#### 1. 改成负的control，会更难reach，但是可以的，去掉damping训练不稳，加质量会好一点，但是更难reach了。
#### 2. Before mj_step() you have qpos(t), qvel(t), xpos(t-1) which may be 0 if t = 0. After mj_step() you have qpos(t+1), qvel(t+1), xpos(t). This is because mj_step() calls mj_forward() to compute everything at time t. Then it advances qpos and qvel to t+1 but does not recompute anything else at t+1.
#### 3. 之前恢复模型文件的时候把sensor搞没了还没发现。
#### 4. mex mujoco给的cpp一直不成功，打不开.mexw64文件，试过改到其他文件夹，加完全控制权限，clear mex，全部文件都在同一个文件夹，mujoco下载下来的文件结构，之前的mex语句，mujoco给的mex语句，DWIN64，都不行，删除就说matlab在占用，只能关掉。
#### 5. tendon在mujoco里不能加质量。
#### 6. 把mjx.cpp里的功能复制到mexstep里可以用，删掉了各种参数检查以及多线程的代码，现在是一个初步的版本，以后有机会再完善，模拟的结果和test2d以及之前的mexstep一致。
#### 7. 之前的mexstep放到sharemujoco文件夹里面了。
## 08/30/2019
### Log:
#### 1. 加joint和tendondamping不改analytical代码仍然work，不确定是不是仍然valid。
#### 2. t2d1的也work，生成的control如果输出成txt再读取，值会变化然后就不work了，save mat文件仍然是work的。解决了，可能是复制的时候出错了，出现了一些奇怪的很大的值，但原control中没有，现在matlab产生的值用文件输出读取两边都work，但是由于round error，和mat save load有一些误差。
#### 3. mujoco可以加skin，看一下！！！！
#### 4. 在closedloop test的时候给模型文件加上control的hard limit还是可以track，试了arm的30%，没看出影响，开环正的改成负的还是会让结果变差一些。
#### 5. 应该把control cost再改小很多，决定设为0。
#### 6. 用strcpy复制字符串到字符串变量，字符串结束符也会被复制，所以即使这个变量之前是很长的字符串，现在复制一个短的进去，由于结束符，这个变量使用的时候会识别成短的而不是短的前面+长的后面。
## 09/02/2019
### Log:
#### 1. test2d里面三个模拟函数内部的state_err等变量需要static类型，不然内存会崩溃。
#### 2. 两个方法umax都用nominal的umax。
#### 3. 算cost的时候都不算control cost。
#### 4. matlab存energy和cost的数组长度要多一个不然影响别的变量。
## 09/04/2019
### Log:
#### 1. python删值的时候第一遍有的没删掉，两个特别大的连在一起，remove第一个之后index变了直接跳过第二个。
## 09/06/2019
### Log:
#### 1. latex figure 不加星号才是单列图，加了就会在整页的中间显示。
#### 2. pre-stress加在swimmer上是ok的，效果相比从零开始确实有提高，但还是训练不成功，游一半不往前了。
#### 3. mujoco加skin！！！
## 09/08/2019
### Log:
#### 1. 个人认为noise大但效果很好的原因一方面是每个string和jointdamping较大，再加上水的阻力，使noise能产生的state deviation减小，再加上timestep小，在有限的时间内即使noise大，state也不会偏太多，刚好还在可控区。
#### 2. 6啊ppt就可以直接导出视频还能配音，最好用arial和times new roman的字体。
#### 3. \setlength{\tabcolsep}{1.7mm}{}调table宽度，可以强行变窄。
## 09/20/2019
### Log:
#### 1. 不能用单位三维向量表示姿态，因为以它自己为轴的旋转无法表示。
## 09/23/2019
### Log:
#### 1. 给quaternion加小扰动然后normalize，旋转是smooth连续的。
#### 2. 先加噪声到nominal的quaternions上，然后做normalization，再用结果减去nominal，就得到了实际加的dx，把这个dx加到nominal的quaternions上赋值给qpos做integration，计算也用这个dx。暂时ok，需要测试。
#### 3. maxstep大了不能运行的原因是大数组不能定义在函数里面，必须在外面定义成全局变量。
## 09/24/2019
### Log:
#### 1. 接上条，sysid3dcheck的误差大约最好的是2.85左右，很大，在小范围调节扰动的方差或quaternions扰动的方差影响不大，把整体扰动或quaternions扰动方差调的很大算出来误差会变得特别大。
#### 2. 现在fish在noise小于7%可以work，openloop差的比较远，效果很明显，可能是max control比较大所以只能承受小点的percentage。
#### 3. 给quaternions赋值会自己变的问题是因为loadmodel的时候写了一个state_nominal+=d->qpos，本来是想在设置了初始state key的时候把设置的初始值变成相对的而不是覆盖，结果导致这个错误，在openloop和test里有，现在都注释了。
#### 4. 不知道到底是不是work了，把quaternions的feedback设成0，结果只变坏一点点，把其他的feedback设成0，结果变坏很多。。。看来还是不行。
## 09/26/2019
### Log:
#### 1. 同样setup，模拟相同step数，RK4比Euler要慢大约2.5倍。
#### 2. 同样setup，RK4能在20% noise下work，Euler大约8%。
## 10/04/2019
### Log:
#### 1. 无噪声mbc能量最小大约是d2c的4倍，继续减小damping ratio和natural frequency就不能reach target了。
## 10/09/2019
### Log:
#### 1. 之前3d的sysid有问题，不能用加减相同扰动的方法。改掉之后full feedback不如完全没有quaternions，如果去掉第三个quat，效果会比较好。但是sysidcheck去掉它error会变大10%。
#### 2. 还是应该多上传github，方便出错了回滚。
#### 3. latex加空行可以用vspace命令，\vspace{10 mm}。
## 10/10/2019
### Log:
#### 1. 用0.002的timestep，sysid的时候增大noise反而error小，但是闭环feedback特别大，不行。
#### 2. contact force归在constraint force里面，d->efc_force, dimension是nefc。contact force perturb不起作用。
## 10/14/2019
### Log:
#### 1. The ball type creates a ball joint with three rotational degrees of freedom. The rotation is represented as a unit quaternion. The quaternion (1,0,0,0) corresponds to the initial configuration in which the model is defined. Any other quaternion is interpreted as a 3D rotation relative to this initial configuration. The rotation is around the point defined by the pos attribute below. If a body has a ball joint, it cannot have other rotational joints (ball or hinge). Combining ball joints with slide joints in the same body is allowed.
#### 2. This attribute specifies the axis of rotation for hinge joints and the direction of translation for slide joints. It is ignored for free and ball joints. The vector specified here is automatically normalized to unit length as long as its length is greater than 10E-14; otherwise a compile error is generated.
#### 3. The joint limits. Limits can be imposed on all joint types except for free joints. For hinge and ball joints, the range is specified in degrees or radians depending on the coordinate attribute of compiler. For ball joints, the limit is imposed on the angle of rotation (relative to the the reference configuration) regardless of the axis of rotation. Only the second range parameter is used for ball joints; the first range parameter should be set to 0. See the Limit section in the Computation chapter for more information.
#### 4. state赋值只赋一部分，剩下的不会自己满足geometry并保持给值的不变。不过给完一部分的值之后step一步然后再重复赋值step，一段时间后剩下的值就会稳定并保证给值了的变量的值是所给的。在这个过程中，剩下的值是从它们原本的值逐渐变到满足geometry的值。
## 10/16/2019
### Log:
#### 1. 3d dbar 9个自由dof，有3个自由quaternions。
## 10/17/2019
### Log:
#### 1. 有了constraint连接之后，非自由的速度不好算，也不能通过强行赋值然后step来让值converge到想设的值，速度设定的话位置没法设定，一定会变，感觉还是得手动计算值然后填入。
## 11/04/2019
### Log:
#### 1. qmc acc 2018的算法，相比bob之前lti系统的qmc，简化了很多，失去了一些guarantee，不再是match前多少个markov parameters而是least square。
#### 2. ltv的主要区别和需要修改的地方，observability和controllability grammian里面不再是A的几次方而是AkAk-1连乘这样，收集数据时一定要按照每一步对应来收集，每一步的A，B，C矩阵size要一致，才可以做state propagation。可以看data matrix Dk的e-value，理论上来说，所有e-value要求是非负的，但是实际上不一定真的都满足，所以去掉负的e-value，看下所有的step正的且值比较大的e-value有多少个，然后依据这个值来取svd后ABC的size。前面建立Hankel matrix和covariance matrix的时候取的q需要保证rank是n，稍微取大点就行，影响不是很大。apply delta u的时候，要保证delta u足够小，系统偏离nominal不太远才可以。
#### 3. coordinate transformation是在得到ABCD后想将其与true system的ABCD比较是否一致时需要做的，在原本的TVERA由于前面几步和中间方法不同，需要merge才能propagate，所以也需要转换坐标，而且算Tk必须知道true system信息才可以，但acc的方法中不需要这样，可以直接用算出来的ABCD做state propagation，得到的Y会match simulation的Y。
#### 4. block shift需要在算数据的时候多算一行的，然后去掉第一行，因为最后一行数据不会凭空产生。
#### 5. 目前初始的delta x取0，每次rollout都从nominal x0开始。
#### 6. pendulum的结果，比较estimation的Y和simulation的Y，趋势一样，值有一些偏差，没有ls的效果好，而且结果有一定随机性，有一些state可能会非常不好。
## 11/06/2019
### Log:
#### 1. 3d的tensegrity模型因为有constraint，导致非自由的joint angle和自由angle之间关系要通过几何计算出来，由于建模的树结构，自由的angle不连在一起，需要特殊处理把他们放一起。
#### 2. QMC加大q会提升准确度，swimmer3效果很不好，只有前100趋势差不多能follow真实系统，没有ls准确。
## 11/11/2019
### Log:
#### 1. 矩阵exp和标量exp不一样，不是每个元素分别求exp，而是矩阵的幂级数，计算时是矩阵相乘。
## 11/17/2019
### Log:
#### 1. 一根bar通过两个joint与地相连，第二个joint是自动连接在第一个joint上的，也就是说，joint1转后，joint2轴会跟着转，但转joint2，joint1轴不变，符合实际。
#### 2. joint的axis属性所设置的轴是在体坐标系下的，会随连接的体而转动。
## 11/19/2019
### Log:
#### 1. camera并不能决定初始时模型的朝向。
#### 2. material文件里material grid的texrepeat越大方格越密。
#### 3. 写review response，文章里的改动用蓝色，对reviewer的response用蓝色，提到文章中的修改用蓝色加加粗。
## 11/23/2019
### Log:
#### 1. matlab可以直接调用exe。
#### 2. 复制扩充矩阵用repmat。
## 11/25/2019
### Log:
#### 1. matlab读取修改xml文件，出现什么什么null是正常的，字符串数组strings(m,n)，setattributes后面的值只能是字符串类型。
## 12/06/2019
### Log:
#### 1. 在tower顶端放一个小球作为payload，可以把小球定义为最上层其中一根bar的子body，中间没有joint连接，不会造成质量集中在一根bar上力不均匀的问题，重力由中心线向下作用在整个结构上。
#### 2. matlab用\代替inv，要把整个inverse以及后面的元素括起来，不然前面如果有乘别的矩阵就会先做乘法整体被求逆，造成维度不匹配。
## 12/10/2019
### Log:
#### 1. 电调上电时不能断掉信号。
#### 2. 电机beep是由电调控制的。
## 12/26/2019
### Log:
#### 1. matlab lqg是stationary的，不直接适用于time varying的情况。
#### 2. vs的命令行窗口在windows搜索里找，用的VS2015 x64 native tools command prompt，直接输入批处理文件名bld_ml64.bat就会自动用ml64执行里面的命令。
## 12/27/2019
### Log:
#### 1. vs调用matlab的教程：https://zhuanlan.zhihu.com/p/23334508。先是将``` C:\Program Files\MATLAB\R2016a\extern\lib\win64; C:\Program Files\MATLAB\R2016a\bin\win64 ``` 加入系统变量，然后在vs工程属性中在 C++->常规->附加包含目录添加：``` C:\Program Files\MATLAB\R2016a\extern\include```,这里的路径应改为matlab的安装目录，在 链接器->常规->附加库目录 添加：```C:\Program Files\MATLAB\R2016a\extern\lib\win64\microsoft```，在链接器->输入->附加依赖库 添加：
```
libmat.lib
libmx.lib
libmex.lib
libeng.lib
```
最后在cpp文件开头加入``` #include <mat.h>```
## 12/28/2019
### Log:
#### 1. 包含头文件时，用户目录下的用双引号，系统目录下的用尖括号。
#### 2. 好像是控制台工程程序算相对路径的起点是vcxproj文件所在目录，直接运行exe的那些起点好像就是exe。
## 12/30/2019
### Log:
#### 1. 宏重定义，原因是在#include "glfw.h" 之前没有#include <windows.h>，mujoco包含glfw是在uitools.h里，不想改uitools文件所以在test.cpp前面#include <windows.h>。
#### 2. windows.h的作用还有getusername这样的函数，所以几个工程的cpp都需要。
#### 3. assert在release版本会默认忽略，只在debug起作用，所以文件是否正常打开这种测试还是使用if做错误检查。
#### 4. sizeof的结果可以用于定义数组维度。
## 01/02/2020
### Log:
#### 1. matlab的reshape排列出的矩阵，c读mat行和列是反的，要注意！！！！！！！！
#### 2. 弄了一个比较简便的顶点处线性化的功能，配合matlab文件可以算出顶点stablizer的gain。在cartpole上实验成功，然后之前cartpole的角度修正少了一个负号，加上就work了。update了一下readme，把3d的描述加上了。
## 01/08/2020
### Log:
#### 1. 修复了testlqg，x1需要每次重新开始rollout的时候置零，不然报错，无法reset，eigen包的矩阵访问index从0开始。
## 01/10/2020
### Log:
#### 1. D一开始就设为0对sysid的影响。把H矩阵中对应D的项设为零，相当于把数值误差泄漏到D上的部分去掉，这样D就是0，符合理论，但是泄漏的误差没有补偿回其他的项，导致其他的项数值不准确，后面计算和svd的时候感觉像是在用残缺的数据。用swimmer测试，把D设为零误差是之前的十多倍，state曲线基本对不上，比不设为零差很多。
#### 2. 一个batch， cartpole的closedloop变差多少。效果很好，不比batch=60差。
#### 3. swimmer的batch起点，用nonzero init，noise必须非常小，因为timestep小导致state error非常小。随机数前面的系数小于e-6左右D的第一个矩阵就不会比后面的大很多。需要更小才可以让第一步error不比后面的step大很多。但是只要这个init很小，batch起始step的CD就会接近零，在那一步观察到的y就会接近是0，解决方法是利用每个batch多id的一步的CD，在一个batch结束时生成后一个batch起始步的y，然后在新batch起始步不生成y，就不会在plot y的时候每次换batch时y都置零，看着很奇怪，但是这样不影响ABCD。
#### 4. 重新做了pendulum和cartpole有无transformation的对比，效果差别不大，有transformation稍微好一点点。
#### 5. nonzero x init的意义在于让xx'更容易等于I，这个是acc2018 paper理论的一部分，如果x初始为0，那前几个CD会很小，A也会有更大误差，这个会影响每个batch的前几步，所以尽量还是不要用zero x init，即使初始值只能很小。
#### 6. swimmer3闭环在前200步效果还可以，后面就跑飞了，10%noise。
#### 7. 既然切换batch的时候CD仍然是0，需要用到前一batch的CD来生成y，是不是应该用前一batch的CD做闭环？对新batch前几步的A会不会有不好影响，可不可以也用上一个batch的？
## 01/13/2020
### Log:
#### 1. sysid的时候增大trial number到4000仍然结果不稳定。
## 01/15/2020
### Log:
#### 1. 增大q到30，error有减小一些。match_q大了会明显变差。
#### 2. 强行让D是0，不管是一开始把h(k,k)置零还是后面只把D设0，结果看起来没有什么差别。
#### 3. B过大会让L过小，算出来闭环控制值非常小，和开环轨迹没区别，让noise大一些，大到id效果不变差，B会比较正常，闭环才有区别，比开环靠近nominal。增大rollout数影响很小。
## 01/19/2020
### Log:
#### 1. tower的sysid结果很不好，下次试下qmc，以及ls缩小timestep。一个很大的问题是设置角度关系时，目前认为相等的角度实际上有误差，结果不准。
## 01/20/2020
### Log:
#### 1. 按位与：a&b是把a和b都转换成二进制数然后再进行与的运算；
逻辑与：a&&b就是当且仅当两个操作数均为 true时，其结果才为 true；只要有一个为零，a&&b就为零。
## 01/23/2020
### Log:
#### 1. extern variable应该在没有extern的那边初始化，但是如果是常量好像两边都得赋值。
## 01/26/2020
### Log:
#### 1. bin文件夹里面放的dll和lib是因为vs的库目录设置的是bin文件夹，然后bin文件夹也是输出exe文件的位置。实际上在exe运行时，exe同文件夹只需要dll就可以。
## 01/27/2020
### Log:
#### 1. 大电脑上gym和其中的mujoco_py包位置D:\Anaconda\Lib\site-packages\gym\envs\mujoco，keras_rl位置是桌面的DDPG_D2C文件夹。如果需要修改就去相应的位置修改，不需要重装包或者reload！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
## 01/30/2020
### Log:
#### 1. ddpg加了process noise好像更好一点，起码pendulum和cartpole是这样的。
## 01/31/2020
### Log:
#### 1. least square的condition number等于最后求Ax=b的x时式子里求逆的那个矩阵的condition number。
## 02/02/2020
### Log:
#### 1. ddpg code 保存文件用的是keras库的callbacks文件，目录在 C:\Users\rwang\Desktop\DDPG_D2C\libraries\keras-rl\rl
#### 2. python zip可以同时循环两个变量。
## 02/03/2020
### Log:
#### 1. python的随机种子设置之后，就决定了出来的随机数，在一次training中没关系，但多次training就会是一样的结果，testing也是的，每次testing要换随机种子。
#### 2. cartpole在0.2fullprocessnoise训练下，0noise test不能把bar摆上去。
#### 3. cartpole umax=12，pendulum umax=5.8，swimmer3 umax=20。
## 02/11/2020
### Log:
#### 1. 认为加terminal controller进horizon之后，最后几步terminal controller启动，控制就不是optimal的了，因为tc是在最后一步设计的最优，即使cost func是一样的，对于前面的步不会最优，所以无noise的error反而变大，但加了noise之后，terminal controller会努力控制到想要到的位置而不是最后一步optimal trajectory的位置，所以对整体有帮助，var和mean都变小。
#### 2. 系统崩溃后会重置到初始位置，error会很大，所以即使崩溃也不会让error比应该的小。
## 02/17/2020
### Log:
#### 1. 布朗运动每一步（t=s1到t=s2）都与之前的无关，是独立的0 mean，正态分布，方差s2-s1。
#### 2. 动量是物体在一个方向运动的趋势，惯性是衡量改变当前运动难易程度。
#### 3. ddpg的deterministic是说在policy生成动作的时候不需要从一个分布采样，而是直接生成一个确定值。
## 02/19/2020
### Log:
#### 1. 
\begin{icmlauthorlist}
\icmlauthor{Ran Wang}{equal,tamuae}
\icmlauthor{Karthikeya S. Parunandi}{equal,tamuae}
\icmlauthor{Dan Yu}{equal,nan}
\icmlauthor{Dileep Kalathil}{equal,tamuee}
\icmlauthor{Suman Chakravorty}{equal,tamuae}
\end{icmlauthorlist}

\icmlaffiliation{tamuae}{Department of Aerospace Engineering, Texas A&M University, Texas, USA}
\icmlaffiliation{tamuee}{Department of Electrical and Computer Engineering, Texas A&M University, Texas, USA}
\icmlaffiliation{nan}{College of Astronautics,
        Nanjing University of Aeronautics and Astronautics, Nanjing, China}

\icmlcorrespondingauthor{Suman Chakravorty}{schakrav@tamu.edu}
## 02/20/2020
### Log:
#### 1. arxiv提交需要点overleaf的log按钮（compile旁边的），拉到最下面，下载bbl文件，改成Main.bbl。
## 03/02/2020
### Log:
#### 1. mujoco里如果加的state noise和constraint有冲突，会先满足state， constraint就变成力了。全是soft constraint。
#### 2. 不记得之前是咋搞的了，现在tower给state noise之后，constraint直接不满足，可能之前是不满足之后再noiseless模拟一段时间然后会慢慢趋向于满足constraint。现在给state noise一定会产生偏转，所以从直上直下推的关系不再成立。
#### 3. 可能是perturb state之后拉歪了然后之后的perturbation关系就会带来constraint break。
#### 4. 因为上层的state是以下层为基础算的，所以下层的变了，上面的state虽然没变但是实际位置被下层带着变了，导致constraint点break。least square应该不适合在mujoco做这种模型的id，凉凉凉凉凉凉~。
#### 5. forward也不会自己去算满足constraint的state，二维自己完全能算，三维很复杂。
## 03/04/2020
### Log:
#### 1. 直上直下关系搞对，第二层bar的perturb自由度去掉，residual e-8，ok的。
## 03/05/2020
### Log:
#### 1. acrobot角度处理是ok的，也能摆上去，但是速度大了稳定不了。
## 03/07/2020
### Log:
#### 1. overleaf段首空格用\hspace{1cm}添加，不过前后必须空行，否则命令无效。
## 03/11/2020
### Log:
#### 1. latex的\author{}里面如果有空行的话，会导致compile一直卡住。
#### 2. 人生第一次公司的电话面试献给了Deeproute.ai，还是紧张了一点，有的地方表达的逻辑不够清晰，总体还可以吧，顺利面完了，没有晕倒。
## 03/25/2020
### Log:
#### 1. mt : general error c101008d: Failed to write the updated manifest to the resource of file "mexstep.mexw64". ?????需要手动删除mexstep.mexw64.manifest文件重新编译。
## 04/20/2020
### Log:
#### 1. matlab的fmincon解有nonlinear constraint的优化，nonlinear constraint function需要有两个输出，分别对应inequality constraint和equality constraint，需要用deal函数分配。
#### 2. 定义函数fun=@(x) x(:,1)+1这种需要特别注意，在需要写fun(x)的时候不能写成fun(x(:,1)),只有定义函数里面可以写成这样，第二个index可以大于1.
## 04/28/2020
### Log:
#### 1. 在写constrained expression的时候用了多阶多项式的话，后面拟合就不能从第一项1开始，但是也不能随便从一个比较后面的项开始，否则residual巨大。
#### 2. figure的label要在end之前caption之后，不然不识别。
## 05/07/2020
### Log:
#### 1. 没有.mat文件的时候，读不到变量，因为结构体没法初始化，后面又有很多调用的地方，所以会直接退出。所以在读的时候如果没文件就把变量结构体初始化成一个全是零但每个成员都有值的状态，dimension那里需要的是const size_t，如果真的用这个不能work，改成static就ok。static mwSize initdimension[3] = { 10, 10, 10 };dataptr->dimension = initdimension;print的时候模式用%d。为了让后面循环里调用的时候index不会越界，第一个dimension设为actuatornum，目前好像就这个限制。现在没有文件也能跑nominal control看效果了，不用再把test也放过来了。
#### 2. 如果从一开始就给noise，就应该从一开始就observe y而不是置零，或者从max(mqx,mqu)+1之后再开始加noise。需要改一下看能不能提高闭环性能。
#### 3. 用matlab读mujoco的数组，site_xpos是3行sitenum列，读sensordata是一行，但是在c里面这些都是vector不是矩阵的。
#### 4. dbar3d full obsv目前0.2 noise work，0.3不行。
#### 5. 写xml模型的时候要注意site的顺序，去掉多余的site，保证最前面的都是不重复的node。
#### 6. openloop直接崩溃没有任何提示，经过尝试，extern const int kMaxStep = 9000; extern const int kMaxState = 60;可以，感觉是内存问题。extern const int kMaxStep = 5000; extern const int kMaxState = 80;也可以，t1d1_3d需要66个。
## 05/08/2020
### Log:
#### 1. tv系统算controllability和observability是用grammian而不是直接一个timestep的矩阵按ti系统的算。
#### 2. 目前找到的主bug是mujoco运行step之后，site和sensor并不会更新，需要手动写一个mjforward在step在函数后面才会更新，所以之前认为的yk一直是yk-1，所以把uk-1改成uk-2反而能对上，只是和TK偏移了一步，但每一个timestep系统变化不大，TK相对于本身的值比较接近，所以尽管偏移一步还是可以work。matlab里面调用mujoco的时候，用的是封装过一遍的step函数，里面自带forward，所以matlab是对齐的。目前认为是这样，希望是对的吧。其实这个知识点早就看到过遇到过，挺久不用忘得一干二净，导致花了一整天的时间找这个bug，而且一开始以为是程序写错了各种试，还是最后老老实实把u，y的nominal打印出来一个一个比较才发现，吸取教训！
## 05/11/2020
### Log:
#### 1. 今天做3d的t2d1 tower,又出现了内存不足直接退出的问题，首先模型文件中nstack不能设的太大。然后局部变量存储的内存stack是比较小的，所以局部变量数组size不能太大，不过不在声明时初始化的局部变量和全局变量会先放在另一个叫BSS的区域，等到运行到给这些变量值的时候再放进stack或data segment。发现函数中所有的if else里面定义的局部变量，是会全部放在stack里的，不会因为之后程序是否进入而取舍，make sense。 全局变量放在data segment里，这个区域size比stack大很多很多，所以不够的时候可以定义成全局，比如static。这个文章讲的挺好的：https://blog.csdn.net/qq_36770641/article/details/88852924.
#### 2. 自动生成tensegrity模型的程序可以运行，不过没有和固定点的连接，里面元素定义顺序也不是最简，用的是free joint加constraint连接，内存使用可以接受，暂时不考虑使用。
## 08/31/2020
### Log:
#### 1. 很久没写了，可能以后直接在ipad上记录了。
#### 2. 为了让Raman能用d2c，修改activate部分。