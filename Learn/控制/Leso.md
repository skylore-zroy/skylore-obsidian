# LESO (线性扩张状态观测器) 详解

## 1. 核心思想

对于任意二阶系统：

$$\ddot{y} + a_1 \dot{y} + a_0 y = b u + d(t)$$

把**除了控制量 $bu$ 之外的所有项**打包成一个"总扰动" $f$：

$$\ddot{y} = \underbrace{\bigl(-a_1 \dot{y} - a_0 y + d(t)\bigr)}_{f(\text{总扰动})} + b u$$

> 关键洞察：不管系统内部参数 ($a_1$, $a_0$) 是否已知、外部扰动 $d(t)$ 是什么形式，统统当作一个扩展状态 $f$ 来处理。

---

## 2. 扩张状态 (Extended State)

定义三个状态变量：

$$x_1 = y, \quad x_2 = \dot{y}, \quad x_3 = f$$

状态空间方程：

$$\begin{cases}
\dot{x}_1 = x_2 \\[4pt]
\dot{x}_2 = x_3 + b_0 u \\[4pt]
\dot{x}_3 = \dot{f} \quad (\text{未知，但 LESO 会估计它}) \\[4pt]
y = x_1
\end{cases}$$

---

## 3. LESO 观测器方程

用输出 $y$ 和输入 $u$ 实时估计三个状态：

$$\begin{cases}
\dot{z}_1 = z_2 + \beta_1 (y - z_1) \\[4pt]
\dot{z}_2 = z_3 + \beta_2 (y - z_1) + b_0 u \\[4pt]
\dot{z}_3 = \phantom{z_4 +}\; \beta_3 (y - z_1)
\end{cases}$$

其中：

- $z_1 \to \hat{y}$（估计输出）
- $z_2 \to \hat{\dot{y}}$（估计速度）
- $z_3 \to \hat{f}$（**估计总扰动** — 这是核心）
- $\beta_1, \beta_2, \beta_3$ = 观测器增益

---

## 4. 带宽参数化 (Gao Zhiqiang, 2003)

把三个增益 $\beta_1, \beta_2, \beta_3$ **绑到一个参数**上——**观测器带宽 $\omega_o$**：

$$\beta_1 = 3\omega_o, \quad \beta_2 = 3\omega_o^2, \quad \beta_3 = \omega_o^3$$

这样 LESO 的特征方程为：

$$(s + \omega_o)^3 = 0$$

所有极点都在 $s = -\omega_o$，系统稳定。**你只需要调一个参数 $\omega_o$**，而不是三个独立的 $\beta$。

---

## 5. 控制律

一旦 LESO 估计出总扰动 $z_3 \approx f$，控制律直接把它**抵消掉**：

$$u = \frac{u_0 - z_3}{b_0}$$

代入系统方程：

$$\ddot{y} = f + b_0 u = f + b_0 \cdot \frac{u_0 - z_3}{b_0} = f + u_0 - \underbrace{z_3}_{\approx f} \approx u_0$$

**扰动被对消了！** 系统退化为纯积分串联型：

$$\ddot{y} \approx u_0$$

然后只需要一个 PD 控制器来跟踪参考信号：

$$u_0 = k_p (r - z_1) + k_d (\dot{r} - z_2)$$

---

## 6. 控制器带宽 $\omega_c$

PD 增益同样用**带宽参数化**绑到一个参数上：

$$k_p = \omega_c^2, \quad k_d = 2\omega_c$$

含义：期望的闭环特征方程为 $(s + \omega_c)^2 = 0$。

---

## 7. 你需要调节的参数 —— 只有 3 个

| 参数             | 含义     | 作用                               |           典型取值范围            |
| -------------- | ------ | -------------------------------- | :-------------------------: |
| **$\omega_c$** | 控制器带宽  | 决定**响应速度**。越大越快，超调也越大            |           4 ~ 20            |
| **$\omega_o$** | 观测器带宽  | 决定**观测精度和速度**。越大观测越准，但噪声敏感       | $3\omega_c \sim 10\omega_c$ |
| **$b_0$**      | 控制增益估计 | 对 $b$ 的粗略估计。不需要精确，LESO 会把误差当扰动吃掉 |       $0.5b \sim 2b$        |

> 这就是 ADRC 最大的工程优势：**不用精确建模，$b_0$ 差几倍都能正常工作**，因为误差会被 $z_3$ 估计并补偿。

---

## 8. 公式汇总

```
┌─────────────────────────────────────────────────────────┐
│  LESO + PD (ADRC) 完整公式                              │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  观测器 (LESO):                                         │
│    ż₁ = z₂ + 3ωo·(y - z₁)                               │
│    ż₂ = z₃ + 3ωo²·(y - z₁) + b₀u                        │
│    ż₃ =         ωo³·(y - z₁)                             │
│                                                         │
│  控制律:                                                │
│    u₀ = ωc²·(r - z₁) + 2ωc·(ṙ - z₂)                     │
│    u  = (u₀ - z₃) / b₀                                  │
│                                                         │
│  调参口诀:                                              │
│    快慢看 ωc，精度看 ωo                                 │
│    ωo = 3~5×ωc 起步，ωo 太小则扰动估计滞后              │
│    b₀ 用粗估值即可                                      │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## 9. 和 PID 的本质区别

| | PID | LESO+PD (ADRC) |
|---|---|---|
| 消除静差 | 靠**积分**（积累历史误差） | 靠**扰动补偿**（实时估计 + 对消） |
| 抗扰 | 扰动 → 误差 → 积分 → 慢慢纠正 | 扰动被 $z_3$ 观测到 → 直接前馈补偿 |
| 积分饱和 | 需要额外处理 | **没有积分环节，不存在 windup** |
| 依赖模型 | 无需模型 | 仅需粗略的 $b_0$ |

---

## 10. 离散化实现 (欧拉法)

仿真中使用的离散形式：

```
对于每个时间步 k:

  控制律:
    u₀(k) = ωc² · (r(k) - z₁(k)) + 2ωc · (ṙ(k) - z₂(k))
    u(k)  = (u₀(k) - z₃(k)) / b₀

  实际系统:
    ddy(k)  = -a₁·dy(k) - a₀·y(k) + b·u(k) + d(k)
    dy(k+1) = dy(k) + ddy(k) · dt
    y(k+1)  = y(k)  + dy(k) · dt

  LESO 更新:
    y_err = y(k) - z₁(k)
    z₁(k+1) = z₁(k) + [z₂(k) + 3ωo · y_err] · dt
    z₂(k+1) = z₂(k) + [z₃(k) + 3ωo² · y_err + b₀·u(k)] · dt
    z₃(k+1) = z₃(k) + [         ωo³ · y_err] · dt
```

---

## 参考文献

- Gao, Z. (2003). Scaling and bandwidth-parameterization based controller tuning. *Proceedings of the American Control Conference*, 4989–4996.
- Han, J. (2009). From PID to Active Disturbance Rejection Control. *IEEE Transactions on Industrial Electronics*, 56(3), 900–906.
