
$$
\begin{align}\tag{40}
 &\min_{\xi_i, \xi_j}\:\sum_{i,j}(p^d_{i}-p_{i,{k+1}})^\top Q_1  (p^d_{i}-p_{i,{k+1}})+ 
\xi_{i,k}^\top Q_2
\xi_{i,k}\\
&\mathrm{s.t.}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) \langle \mathrm{grad}_T\:P_i^l,\xi_i\rangle
\\&\qquad+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\langle \mathrm{grad}_T\:P_j^l,\xi_j\rangle\\
&\qquad < \gamma_0 (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^k))
\end{align}
$$
