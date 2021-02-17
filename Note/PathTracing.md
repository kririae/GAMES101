# Path Tracing

在 GAMES101 里学了一次 Path Tracing，感觉并不够。还是得自己来一次。

## Radiometry

1. 光子的能量为 $q = h f = h c/\lambda$，高中物理。但是需要注意的是，在 SI unit 里描述波长的量是 *nanometer*。很有意思的一点是，若要对波长进行“积分”，很可能需要违背其离散的物理意义。因此引入的新单位 *spectral energy* $\mathrm{J(nm)^{-1}}$，是能量关于波长的微分，记作$Q_\lambda$，再延伸为 $\Phi_{\lambda} \mathrm{W(nm)^{-1}}$。（吐槽一下，国外教材真的讲的好细……）

2. *Power*(W, Joules/s) 是能量的单位。若对每个光子进行模拟，$100\mathrm{W}, \lambda = 50\mathrm{nm}$ 的电灯泡会产生约$10^{20}$个光子，基本不可模拟。

3. **Irradiance**（似乎是翻译为照度），这个单位用于回答"How much light hits this point"。于是我们在这里 Spectral Power 的单位上再对面积进行微分，即
   $$
   H = \frac{\Delta\Phi}{\Delta A} = \frac{\Delta q}{\Delta A \Delta t \Delta \lambda}
   $$

4. **Radiance**（辐射）。其被定义的原因是，光线与某个面作用的时候，还需要考虑作用的方向。Irradiance 只能用于衡量作用的总量，但是不能衡量作用的方向。那，再对立体角进行一次微分即可。即 $\Delta H / \Delta \sigma$。其物理意义大致是 光线与某个面作用时，在某个方向上的能量。教材上的图示似乎有点问题… 因为 $\Delta A$ 是针对我们使用的表面定义的，假设这个表面是水平的，而我们在水平的表面上接收到的是 $r$，那么原本射入的 radiance 应该更大才对，即我们使用的 $r/\cos\theta$， $\Delta H / (\Delta \sigma \cos \theta)$

   （上面这段我折腾了好久，最后总结起来就是一句话。radiance 的定义是针对平面的）

   然后我们定义两个东西，$L_s$ 为 surface radiance，表示“离开”某个 surface 的 radiance，而 $L_f$ 表示进入某个表面的 radiance。

5. $$
   H = \int_{\mathrm{all} \ \bold{k}}{L_f(\bold{k}) \cos \theta  \ \mathrm{d}\sigma}
   $$
   （顺带提醒一下，$H$ 表示进入，$E$ 表示离开）

   这个公式说明了，进入某个面的 irradiance 是这个面的 field radiance 的积分。$\bold{k}$ 表示所有接近进入这个面的向量。将公式进行二元积分得到，在每个方向上 field radiance 相同的情况下 $H = \pi L_f$（懒得抄公式了）
   
   类似地，也可以积分出 $\Phi = \displaystyle\int_{\mathrm{all} \  \bold{x}}{H(\bold{x}) \ \mathrm{d} A}$，可以在一定条件下从 radiance 一直得到 power.
   
### BRDF

   > > We would like to characterize how a surface reflects light.
   > >
   > > > 对物理世界的精确或者不精确抽象构成了计算机图形学

这便是 BRDF 的动机。目前我们都是基于 $\Delta A$ 在考虑，则 BRDF 的最基本能完成一个 $(\bold{k}_i, \bold{k}_o)$ 的映射。

我们应用比值定义一个函数
$$
\rho = \frac{L_s}{H}
$$
而这个比值 $\rho$ 会随着 $(\bold{k}_i, \bold{k}_o)$ 的变化而变化。这就是 BRDF，*bidirectional reflectance distribution function*。

定义 *directional hemispherical reflectance(定向半球反射)* 为
$$
R(\bold{k}_i) = \frac{\text{power of all the corresponding } \bold{k}_o}{\text{in power of a certain } \bold{k}_i}
$$
且 $R(\bold{k}_i) \in [0, 1]$，因为可能存在能量吸收的问题。

此处略过一个推导，
$$
R(\bold{k}_i) = \int_{\mathrm{all} \ \bold{k}_0}{\rho(\bold{k}_i, \bold{k}_o) \cos \theta_o \ \mathrm{d} \sigma_o}
$$
即定向半球反射率是 BRDF 在所有方向上的积分。

#### Ideal Diffuse BRDF

在 Lambertian BRDF 模型中，$\rho \equiv C$，于是我们积分得到 $R(\bold{k}_i) = \pi C$。

## Transport Equation

> >  [TODO]