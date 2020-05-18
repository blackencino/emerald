
# Artificial rest density {rho_r}^0
This is always 1, by construction and definition. Why do the equations still use a symbolic
representation for it? Are there situations where it would be variable and set to something
other than 1?

# The source term s_r
When you compute the source term s_r, it is computed by (({rho_r}^0 - rho_r)/dt) + rho_r * divergence.  But divergence is computed as a sum times (1/rho_r). Those divergences don't
seem to be used anywhere else - could we just skip dividing the divergence by rho_r, just to
multiply it by rho_r again?

# Appendix, algorithm B.1
In line 2, gradient is over all neighboring particles, or just particles of adjacent bodies?
(Compute gradient Grad {p_r}^b = Sum_{r_k} V_{r_k} rho_{r_k} 1/{rho_{r_k}}^2 Grad {W_{rr_k}}).
It seems, based on walking this through and the r_k subscript that it should be only neighbor
particles of other bodies.

In line 3, when computing the linear and angular velocities of the rigid body R from the
pressure gradient computed in line 2, does the linear and angular velocity include the
existing linear and angular velocity from the previous state of the system and external
forces, or is it *just* the linear and angular velocities caused by this individual
pressure gradient? It seems like all of the things with the 'b' superscript are just the
incremental differences, almost like impulses, rather than incorporating the external
velocities.

If you skip lines 6 & 7, then the {v_{r_k}}^b term is just the same {v_r}^b as computed
in line 3 (but at the other index k), and therefore line 9 is just the negative divergence
of v^b, correct? Or, does the {v_{r_k}}^b term just become zero if lines 6 & 7 are deleted?
