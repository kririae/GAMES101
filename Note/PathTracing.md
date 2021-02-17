# Path Tracing

在 GAMES101 里学了一次 Path Tracing，感觉并不够。还是得自己来一次。

## Radiometry

1. 光子的能量为 $q = h f = h c/\lambda$，高中物理。但是需要注意的是，在 SI unit 里描述波长的量是 *nanometer*。很有意思的一点是，若要对波长进行“积分”，很可能需要违背其离散的物理意义。因此引入的新单位 *spectral energy* $\mathrm{J(nm)^{-1}}$，是能量关于波长的微分，记作$Q_\lambda$，再延伸为 $\Phi_{\lambda} \mathrm{W(nm)^{-1}}$。（吐槽一下，国外教材真的讲的好细……）

2. *Power*(W, Joules/s) 是能量的单位。若对每个光子进行模拟，$100\mathrm{W}, \lambda = 50\mathrm{nm}$ 的电灯泡会产生约$10^{20}$个光子，基本不可模拟。

3. **Irradiance**（似乎是翻译为照度），这个单位用于回答"How much light hits this point"。于是我们在这里 Spectral Power 的单位上再对面积进行微分，即
   $$
   H = \frac{\Delta\Phi}{\Delta A} = \frac{\Delta q}{\Delta A \Delta t \Delta \lambda}
   $$

4. **Radiance**（辐射）。其被定义的原因是，光线与某个面作用的时候，还需要考虑作用的方向。Irradiance 只能用于衡量作用的总量，但是不能衡量作用的方向。那，再对立体角进行一次微分即可。即 $\Delta H / \Delta \sigma$。其物理意义大致是 光线与某个面作用时，在某个方向上的能量。**但是有个问题**，即：侧面入射的光线，一定会被“某个面”完全吸收吗？必然是不可能的，转化为 $\Delta H / (\Delta \sigma \cos \theta)$