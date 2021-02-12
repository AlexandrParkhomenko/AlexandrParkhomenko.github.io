# Multi-Body Dynamics

# Deriving the equations of motion (an example)

The equations of motion for a standard robot can be derived using the
method of Lagrange.  Using $T$ as the total kinetic energy of the system,
and $U$ as the total potential energy of the system, $L = T-U$, and $Q_i$ as
the generalized force corresponding to $q_i$, the Lagrangian dynamic
equations are:

$$\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} - \frac{\partial L}{\partial q_i} = Q_i.$$

If you are not comfortable with these equations, then
any good book chapter on rigid body mechanics can bring you up to speed --
try <elib>Craig89</elib> for a very practical guide to robot
kinematics/dynamics, <elib>Goldstein02</elib> for a more hard-core dynamics
text or <elib>Thornton03</elib> for a classical dynamics text which is a
nice read -- for now you can take them as a handle that you can crank to
generate equations of motion.

# Simple Double Pendulum

<figure>
<img style="width:250px;" src="figures/simple_double_pend.svg"/>
<figcaption>Simple double pendulum</figcaption>
</figure>

Consider the simple double pendulum with torque actuation at both
joints and all of the mass concentrated in two points (for simplicity).
Using ${\bf q}  = [\theta_1,\theta_2]^T$, and ${\bf p}_1,{\bf p}_2$ to denote the locations of $m_1,m_2$, respectively, the kinematics of this system
are:

\begin{eqnarray*}
{\bf p}_1 =& l_1\begin{bmatrix} s_1 \\ - c_1 \end{bmatrix}, &{\bf p}_2  =
{\bf p}_1 + l_2\begin{bmatrix} s_{1+2} \\ - c_{1+2} \end{bmatrix} \\
\dot{{\bf p}}_1 =& l_1 \dot{q}_1\begin{bmatrix} c_1 \\ s_1 \end{bmatrix},
&\dot{{\bf p}}_2 = \dot{{\bf p}}_1 + l_2 (\dot{q}_1+\dot{q}_2) \begin{bmatrix} c_{1+2} \\ s_{1+2} \end{bmatrix}
\end{eqnarray*}

Note that $s_1$ is shorthand for $\sin(q_1)$, $c_{1+2}$ is shorthand for
$\cos(q_1+q_2)$, etc. From this we can write the kinetic and potential
energy:

\begin{align*}
T =& \frac{1}{2} m_1 \bf{\dot p}_1^T \bf{\dot p}_1 + \frac{1}{2} m_2
\bf{\dot p}_2^T \bf{\dot p}_2 \\
=& \frac{1}{2}(m_1 + m_2) l_1^2 \dot{q}_1^2 + \frac{1}{2} m_2 l_2^2 (\dot{q}_1 + \dot{q}_2)^2 + m_2 l_1 l_2 \dot{q}_1 (\dot{q}_1 + \dot{q}_2) c_2 \\
U =& m_1 g y_1 + m_2 g y_2 = -(m_1+m_2) g l_1 c_1 - m_2 g l_2 c_{1+2}
\end{align*}

Taking the partial derivatives $\frac{\partial T}{\partial q_i}$, $\frac{\partial T}{\partial \dot{q}_i}$, and
$\frac{\partial U}{\partial q_i}$ ($\frac{\partial U}{\partial \dot{q}_i}$ terms are always zero), then
$\frac{d}{dt}\frac{\partial T}{\partial \dot{q}_i}$, and plugging them into the Lagrangian,
reveals the equations of motion:

\begin{align*}
(m_1 + m_2) l_1^2 \ddot{q}_1& + m_2 l_2^2 (\ddot{q}_1 + \ddot{q}_2) + m_2 l_1 l_2 (2 \ddot{q}_1 + \ddot{q}_2) c_2 \\
&- m_2 l_1 l_2 (2 \dot{q}_1 + \dot{q}_2) \dot{q}_2 s_2 + (m_1 + m_2) l_1 g s_1 + m_2 g l_2 s_{1+2} = \tau_1 \\
m_2 l_2^2 (\ddot{q}_1 + \ddot{q}_2)& + m_2 l_1 l_2 \ddot{q}_1 c_2 + m_2 l_1 l_2
\dot{q}_1^2 s_2 + m_2 g l_2 s_{1+2} = \tau_2
\end{align*}

# The Manipulator Equations

If you crank through the Lagrangian dynamics for a few simple robotic
manipulators, you will begin to see a pattern emerge - the resulting
equations of motion all have a characteristic form.  For example, the
kinetic energy of your robot can always be written in the form:
\begin{equation} T = \frac{1}{2} \bf{\dot q}^T \bf M(\bf q)
\bf{\dot q},\end{equation} where $\bf M$ is the state-dependent inertia matrix
(aka mass matrix). This observation affords some insight into general
manipulator dynamics - for example we know that ${\bf M}$ is always positive
definite, and symmetric<elib>Asada86</elib>(p.107) and has a beautiful
sparsity pattern<elib>Featherstone05</elib> that we'll attempt to take
advantage of in our algorithms.


Continuing our abstractions, we find that the equations of motion of a
general robotic manipulator (without kinematic loops) take the form

\begin{equation} \bf{M}(\bf{q})\bf{\ddot q} + \bf{C}(\bf{q},\bf{\dot q})\bf{\dot q} =
\bf \tau_g(\bf q) + {\bf B}\bf u, \label{eq:manip} \end{equation}

where $\bf q$ is
the joint position vector, ${\bf M}$ is the inertia matrix, $\bf C$ captures
Coriolis forces, and $\bf \tau_g$ is the gravity vector.  The matrix $\bf B$ maps
control inputs $\bf u$ into generalized forces.  Note that we pair
$\bf{M \ddot{q}} + \bf{C \dot q}$ on the left side because "... the equations of
motion depend on the choice of coordinates $\bf q$.  For this reason neither
$\bf{M \ddot q}$ nor $\bf{C\dot q}$ individually should be thought of as a generalized force; only their sum is a force"<elib>Choset05</elib>(s.10.2).
Indeed, whenever I write Eq. (\ref{eq:manip}), I see $ma = F$. 

# Manipulator Equation form of the Simple Double Pendulum

The equations of motion from Example 1 can be written
compactly as: \begin{align*} \bf M(\bf q) =& \begin{bmatrix} (m_1 + m_2)l_1^2 +
m_2 l_2^2 + 2 m_2 l_1l_2 c_2 & m_2 l_2^2 + m_2 l_1 l_2 c_2 \\ m_2 l_2^2 +
m_2 l_1 l_2 c_2 & m_2 l_2^2 \end{bmatrix} \\ \bf C(\bf q,\bf{\dot q}) =&
\begin{bmatrix} 0 & -m_2 l_1 l_2 (2\dot{q}_1 + \dot{q}_2)s_2 \\ m_2 l_1 l_2
\dot{q}_1 s_2 & 0 \end{bmatrix} \\ \bf \tau_g(\bf q) =& -g \begin{bmatrix} (m_1 +
m_2) l_1 s_1 + m_2 l_2 s_{1+2} \\ m_2 l_2 s_{1+2} \end{bmatrix} , \quad \bf B
= \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix} \end{align*} Note that this
choice of the $\bf C$ matrix was not unique. </example>

The manipulator equations are very general, but they do define some
important characteristics.  For example, $\bf{\ddot q}$ is (state-dependent)
linearly related to the control input, $\bf u$.   This observation justifies
the control-affine form of the dynamics assumed throughout the notes.

Note that we have chosen to use the notation of second-order systems
(with $\bf{\dot q}$ and $\bf{\ddot q}$ appearing in the equations) throughout
this book.  Although I believe it provides more clarity, there is an
important limitation to this notation: it is impossible to describe 3D
rotations in "minimal coordinates" using this notation without introducing
kinematic singularities (like the famous "gimbal lock"). For instance, a
common singularity-free choice for representing a 3D rotation is the unit
quaternion, described by 4 real values (plus a norm constraint).  However we
can still represent the rotational velocity without singularities using just
3 real values.  This means that the length of our velocity vector is no
longer the same as the length of our position vector.  For this reason, you
will see that most of the software in <drake></drake> uses the more general
notation with $\bf v$ to represent velocity, $\bf q$ to represent positions, and
the manipulator equations are written as \begin{equation} \bf M(\bf q) \bf{\dot v}
+ \bf C(\bf q,\bf v)\bf v = \bf \tau_g(\bf q) + \bf B \bf u, \end{equation} which is
not necessarily a second-order system.  See <elib>Duindam06</elib> for a
nice discussion of this topic.

# Recursive Dynamics Algorithms

The equations of motions for our machines get complicated quickly.
Fortunately, for robots with a tree-link kinematic structure, there are
very efficient and natural recursive algorithms for generating these
equations of motion.  For a detailed reference on these methods, see
<elib>Featherstone07</elib>; some people prefer reading about the
Articulated Body Method in <elib>Mirtich96</elib>.  The implementation in
<drake></drake> uses a related formulation from <elib>Jain11</elib>.

</subsection>

# Bilateral Position Constraints

If our robot has closed-kinematic chains, for instance those that arise
from a <a href="https://en.wikipedia.org/wiki/Four-bar_linkage">four-bar
linkage</a>, then we need a little more.  The Lagrangian machinery above
assumes "minimal coordinates"; if our state vector $\bf q$ contains all of
the links in the kinematic chain, then we do not have a minimal
parameterization -- each kinematic loop adds (at least) one constraint so
should remove (at least) one degree of freedom.  Although some constraints
can be solved away, the more general solution is to use the Lagrangian to
derive the dynamics of the unconstrained system (a kinematic tree without
the closed-loop constraints), then add additional generalized forces that
ensure that the constraint is always satisfied. 

Consider the constraint equation

\begin{equation}\bf h(\bf q) =
0.\end{equation}
For the case of the kinematic closed-chain, this can be
the kinematic constraint that the location of one end of the chain is
equal to the location of the other end of the chain. The equations of
motion can be written \begin{equation}\bf M({\bf q})\bf{\ddot q} +
\bf C(\bf q,\bf{\dot q})\dot\bf q = \bf \tau_g(\bf q) + \bf B\bf u + \bf H^T(\bf q)
\bf \lambda,\label{eq:constrained_manip}\end{equation} where $\bf H(\bf q) = \frac{\partial \bf h}{\partial \bf q}$ and ${\bf \lambda}$ is the constraint force.  Let's use the
shorthand 
\begin{equation} \bf M({\bf q})\bf{\ddot q} = \bf \tau(q,\dot{q},u) + \bf H^T(\bf q) \bf \lambda. \label{eq:manip_short} \end{equation}<

Using 

\begin{gather}\bf{\dot h} = \bf{H \dot q},\\ \bf{\ddot h} = \bf{H \ddot q} +
\bf{\dot H \dot q}, \label{eq:ddoth} \end{gather}

we can solve for
$\bf \lambda$, by observing that when the constraint is imposed, $\bf h=0$ and
therefore $\bf \dot h=0$ and $\bf\ddot h=0$.  Inserting the dynamics
(\ref{eq:manip_short}) into (\ref{eq:ddoth}) yields
\begin{equation}\bf \lambda =- (\bf H \bf M^{-1} \bf H^T)^+ (\bf H \bf M^{-1} \bf \tau +
\bf \dot H\bf \dot q).\label{eq:lambda_from_h}\end{equation} The $^+$ notation refers to a Moore-Penrose
pseudo-inverse.  In many cases, this matrix will be full rank (for
instance, in the case of multiple independent four-bar linkages) and the
traditional inverse could have been used.  When the matrix drops rank
(multiple solutions), then the pseudo-inverse will select the solution
with the smallest constraint forces in the least-squares sense.

To combat numerical "constraint drift", one might like to add a
restoring force in the event that the constraint is not satisfied to
numerical precision.  To accomplish this, rather than solving for
$\bf \ddot h = 0$ as above, we can instead solve for 
\begin{equation}\bf\ddot h
= \bf H\bf\ddot q + \bf{\dot H \dot q} = -2\alpha \bf\dot h - \alpha^2
\bf h,\end{equation}
where $\alpha>0$ is a stiffness parameter. This is
known as Baumgarte's stabilization technique, implemented here with a
single parameter to provide a critically-damped response. Carrying this
through yields \begin{equation} \bf \lambda =- (\bf H \bf M^{-1} \bf H^T)^+ (\bf H
\bf M^{-1} \bf\tau + (\bf{\dot H} + 2\alpha \bf H)\bf{\dot q} + \alpha^2 \bf H).
\end{equation}

There is an important optimization-based derivation/interpretation of
these equations, which we will build on below, using <a
href="https://en.wikipedia.org/wiki/Gauss%27s_principle_of_least_constraint">Gauss's
Principle of Least Constraint</a>.  This principle says that we can
derive the constrained dynamics as : 

\begin{align} \min_\bf{\ddot q} \quad
& \frac{1}{2} (\bf{\ddot q} - \bf{\ddot q}_{uc})^T \bf M (\bf{\ddot q} -
\bf{\ddot q}_{uc}), \label{eq:least_constraint} \\ \text{subject to} \quad &
\bf{\ddot H}(\bf q,\dot\bf q,\bf\ddot q) = 0 = \bf H \bf{\ddot q} + \dot\bf H \bf{\dot q},
\nonumber \end{align}

where $\bf{\ddot q}_{uc} = \bf{M^{-1}}\tau$ is the "unconstrained acceleration"
<elib>Udwadia92</elib>. Equation \eqref{eq:constrained_manip} comes right
out of the optimality conditions with $\lambda$ acting precisely as the
Lagrange multiplier.  This is a convex quadratic program with equality
constraints, which has a closed-form solution that is precisely Equation
\eqref{eq:lambda_from_h} above.  It is also illustrative to look at the
dual formulation of this optimization, which can be written as

$$\min_\lambda \frac{1}{2} \lambda^T \bf H \bf M^{-1} \bf H^T \lambda -
\lambda^T (\bf H \bf{M^{-1}}\bf\tau + \bf{\dot H} \bf{\dot q}).$$

Observe that the
linear term is contains the acceleration of $\bf H$ if the dynamics evolved
without the constraint forces applied; let's call that $\bf{\ddot H}_{uc}$:
$$\min_\lambda \frac{1}{2} \lambda^T \bf H \bf M^{-1} \bf H^T \lambda -
\lambda^T \bf{\ddot H}_{uc}.$$
The primal formulation has accelerations as
the decision variables, and the dual formulation has constraint forces as
the decision variables.  For now this is merely a cute observation; we'll
build on it more when we get to discussing contact forces.

</subsection>

# Bilateral Velocity Constraints

Consider the constraint equation \begin{equation}\bf H_v(\bf q,\bf{\dot q}) =
0,\end{equation} where $\frac{\partial \bf H_v}{\partial \bf{\dot q}} \ne 0.$  These are less
common, but arise when, for instance, a joint is driven through a
prescribed motion. Here, the manipulator equations are given by

\begin{equation}\bf M\bf{\ddot q}  = \bf\tau + \frac{\partial \bf H_v}{\partial \bf{\dot q}}^T
\bf \lambda.\end{equation}

Using

\begin{equation} \bf\dot H_v = \frac{\partial \bf H_v}{\partial \bf q}
\bf{\dot q} + \frac{\partial \bf H_v}{\partial \bf{\dot q}}\bf{\ddot q},\end{equation}

setting

$\bf\dot H_v = 0$ yields \begin{equation}\bf \lambda = - \left(
\frac{\partial \bf H_v}{\partial \bf{\dot q}} \bf M^{-1} \frac{\partial \bf H_v}{\partial \bf{\dot q}} \right)^+ \left[
\frac{\partial \bf H_v}{\partial \bf{\dot q}} \bf M^{-1} \bf\tau + \frac{\partial \bf H_v}{\partial \bf q} \bf{\dot q}
\right].\end{equation}

Again, to combat constraint drift, we can ask instead for $\dot\bf H_v =
-\alpha \bf H_v$, which yields \begin{equation}\bf \lambda = - \left(
\frac{\partial \bf H_v}{\partial \bf{\dot q}} \bf M^{-1} \frac{\partial \bf H_v}{\partial \bf{\dot q}} \right)^+ \left[
\frac{\partial \bf H_v}{\partial \bf{\dot q}} \bf M^{-1} \bf\tau + \frac{\partial \bf H_v}{\partial \bf q} \bf{\dot q} + \alpha
\bf H_v \right].\end{equation}

</subsection>

</section>

# The Dynamics of Contact

The dynamics of multibody systems that make and break contact are closely
related to the dynamics of constrained systems, but tend to be much more
complex.  In the simplest form, you can think of non-penetration as an
<i>inequality</i> constraint: the signed distance between collision bodies
must be non-negative.  But, as we have seen in the chapters on walking, the
transitions when these constraints become active correspond to collisions,
and for systems with momentum they require some care.  We'll also see that
frictional contact adds its own challenges.

There are three main approaches used to modeling contact with "rigid"
body systems:  1) rigid contact approximated with stiff compliant contact,
2) hybrid models with collision event detection, impulsive reset maps, and
continuous (constrained) dynamics between collision events, and 3) rigid
contact approximated with time-averaged forces (impulses) in a time-stepping
scheme. Each modeling approach has advantages/disadvantages for different
applications.

Before we begin, there is a bit of notation that we will use throughout.
Let $\phi(\bf q)$ indicate the relative (signed) distance between two rigid
bodies.  For rigid contact, we would like to enforce the unilateral
constraint: \begin{equation} \phi(\bf q) \geq 0. \end{equation}  We'll use
${\bf n} = \frac{\partial \phi}{\partial \bf q}$ to denote the "contact normal", as well as ${\bf
t}_1$ and ${\bf t}_2$ as a basis for the tangents at the contact
(orthonormal vectors in the Cartesian space, projected into joint space),
all represented as row vectors in joint coordinates.  


<figure><img width="100%" src="figures/contact_coordinates_2d.jpg"
/><figcaption>Contact coordinates in 2D.  (a) The signed distance between
contact bodies, $\phi(\bf q)$. (b) The normal (${\bf n}$) and tangent (${\bf
t}$) contact vectors -- note that these can be drawn in 2D when the $\bf q$ is
the $x,y$ positions of one of the bodies, but more generally the vectors
live in the configuration space. (c) Sometimes it will be helpful for us to
express the tangential coordinates using $d_1$ and $d_2$; this will make
more sense in the 3D case.</figcaption></figure>

<figure><img width="100%" src="figures/contact_coordinates_3d.jpg"
/><figcaption>Contact coordinates in 3D.</figcaption></figure>

We will also find it useful to assemble the contact normal and tangent
vectors into a single matrix, $\bf J$, that we'll call the <i>contact
Jacobian</i>:

$$\bf J(\bf q) = \begin{bmatrix} {\bf n}\\ {\bf t}_1\\ {\bf t}_2
\end{bmatrix}.$$

As written, $\bf J\bf v$ gives the relative velocity of the
closest points in the contact coordinates; it can be also be extended with
three more rows to output a full spatial velocity (e.g. when modeling
torsional friction).  The generalized force produced by the contact is given
by $\bf J^T \lambda$, where $\lambda = [f_n, f_{t1}, f_{t2}]^T:$
\begin{equation} \bf M(\bf q) \dot{\bf v} + \bf C(\bf q,\bf v)\bf v = \bf\tau_g(\bf q) + \bf B
\bf u + \bf J^T(\bf q)\lambda. \end{equation}

# Compliant Contact Models

Most compliant contact models are conceptually straight-forward: we
will implement contact forces using a stiff spring (and
damper<elib>Hunt75</elib>) that produces forces to resist penetration (and
grossly model the dissipation of collision and/or friction).  For
instance, for the normal force, $f_n$, we can use a simple (piecewise)
linear spring law:

<figure>
<img width="50%" src="figures/contact_spring.jpg" />
</figure>

Coulomb friction is described by two parameters -- $\mu_{static}$ and
$\mu_{dynamic}$ -- which are the coefficients of static and dynamic
friction.  When the contact tangential velocity (given by ${\bf t}\bf v$) is
zero, then friction will produce whatever force is necessary to resist
motion, up to the threshold $\mu_{static} f_n.$ But once this threshold is
exceeding, we have slip (non-zero contact tangential velocity); during
slip friction will produce a constant drag force $|f_t| = \mu_{dynamic}
f_n$ in the direction opposing the motion.  This behaviour is not a simple
function of the current state, but we can approximate it with a continuous
function as pictured below.

<figure>
<img width="90%" src="figures/contact_stribeck.jpg" />
<figcaption>(left) The Coloumb friction model.  (right) A continuous piecewise-linear approximation of friction (green) and the <a href="https://drake.mit.edu/doxygen_cxx/group__stribeck__approximation.html">Stribeck approximation of Coloumb friction</a> (blue); the $x$-axis is the contact tangential velocity, and the $y$-axis is the friction coefficient.</figcaption>
</figure>

With these two laws, we can recover the contact forces as relatively
simple functions of the current state.  However, the devil is in the
details.  There are a few features of this approach that can make it
numerically unstable. If you've ever been working in a robotics simulator
and watched your robot take a step only to explode out of the ground and
go flying through the air, you know what I mean.  

In order to tightly approximate the (nearly) rigid contact that most
robots make with the world, the stiffness of the contact "springs" must be
quite high.  For instance, I might want my 180kg humanoid robot model to
penetrate into the ground no more than 1mm during steady-state standing.
The challenge with adding stiff springs to our model is that this results
in <a href="https://en.wikipedia.org/wiki/Stiff_equation">stiff
differential equations</a> (it is not a coincidence that the word
<em>stiff</em> is the common term for this in both springs and ODEs). As a
result, the best implementations of compliant contact for simulation make
use of special-purpose numerical integration recipes (e.g.
<elib>Castro20</elib>) and compliant contact models are often difficult to
use in e.g. trajectory/feedback optimization.

But there is another serious numerical challenge with the basic
implementation of this model.  Computing the signed-distance function,
$\phi(\bf q)$, when it is non-negative is straightforward, but robustly
computing the "penetration depth" once two bodies are in collision is not.
Consider the case of use $\bf phi(\bf q)$ to represent the maximum penetration
depth of, say, two boxes that are contacting each other.  With compliant
contact, we must have some penetration to produce any contact force.  But
the direction of the normal vector, ${\bf n} = \frac{\partial \bf phi}{\partial \bf q},$ can
easily change discontinuously as the point of maximal penetration moves,
as I've tried to illustrate here:

<figure><img width=90% src="figures/contact_penetration_normals.jpg" />
<figcaption>(Left) Two boxes in penetration, with the signed distance
defined by the maximum penetration depth.  (Right) The contact normal for
various points of maximal penetration.
</figcaption></figure>

If you really think about this example, it actually even more of the
foibles of trying to even define the contact points and normals
consistently.  It seems reasonable to define the point on body B as the
point of maximal penetration into body A, but notice that as I've drawn
it, that isn't actually unique!  The corresponding point on body A should
probably be the point on the surface with the minimal distance from the
max penetration point (other tempting choices would likely cause the
distance to be discontinuous at the onset of penetration).  But this whole
strategy is asymmetric;  why shouldn't I have picked the vector going with
maximal penetration into body B?  My point is simply that these cases
exist, and that even our best software implementations get pretty
unsatisfying when you look into the details.  And in practice, it's a lot to expect the collision engine to give consistent and smooth contact points out in every situation.

Some of the advantages of this approach include (1) the fact that it is
easy to implement, at least for simple geometries, (2) by virtue of being
a continuous-time model it can be simulated with error-controlled
integrators, and (3) the tightness of the approximation of "rigid" contact
can be controlled through relatively intuitive parameters.  However, the
numerical challenges of dealing with penetration are very real, and they
motivate our other two approaches that attempt to more strictly enforce
the non-penetration constraints.

</subsection>

# Rigid Contact with Event Detection

# Impulsive Collisions

The collision event is described by the zero-crossings (from positive
to negative) of $\phi$. Let us start by assuming frictionless
collisions, allowing us to write \begin{equation}\bf M\bf{\ddot q} = \bf\tau +
\lambda {\bf n}^T,\end{equation} where ${\bf n}$ is the "contact normal"
we defined above and $\lambda \ge 0$ is now a (scalar) impulsive force
that is well-defined when integrated over the time of the collision
(denoted $t_c^-$ to $t_c^+$). Integrate both sides of the equation over
that (instantaneous) interval: \begin{align*}\int_{t_c^-}^{t_c^+}
dt\left[\bf M \bf{\ddot q} \right] = \int_{t_c^-}^{t_c^+} dt \left[ \bf\tau +
\lambda {\bf n}^T \right] \end{align*} Since $\bf M$, $\bf\tau$, and ${\bf
n}$ are constants over this interval, we are left with

$$\bf M\bf{\dot q}^+- \bf M\bf{\dot q}^- = {\bf n}^T \int_{t_c^-}^{t_c^+} \lambda dt,$$

where
$\bf{\dot q}^+$ is short-hand for $\bf{\dot q}(t_c^+)$. Multiplying both
sides by ${\bf n} \bf M^{-1}$, we have

$$ {\bf n} \bf{\dot q}^+ - {\bf
n}\bf{\dot q}^- = {\bf n} \bf M^{-1} {\bf n}^T \int_{t_c^-}^{t_c^+}
\lambda dt.$$

After the collision, we have $\dot\phi^+ = -e \dot\phi^-$,
with $0 \le e \le 1$ denoting the <a
href="https://en.wikipedia.org/wiki/Coefficient_of_restitution">
coefficient of restitution</a>, yielding:

$$ {\bf n} \bf{\dot q}^+ -
{\bf n}\bf{\dot q}^- = -(1+e){\bf n}\bf{\dot q}^-,$$

and therefore

$$\int_{t_c^-}^{t_c^+} \lambda dt = - (1+e)\left[ {\bf n} \bf M^{-1}
{\bf n}^T \right]^\# {\bf n} \bf{\dot q}^-.$$

I've used the notation
$A^\#$ here to denote the pseudoinverse of $A$ (I would normally write
$A^+,$ but have changed it for this section to avoid confusion).
Substituting this back in above results in \begin

{equation}\bf{\dot q}^+ =
\left[ \bI - (1+e)\bf M^{-1} {\bf n}^T \left[{\bf n} \bf M^{-1} {\bf n}^T
\right]^\# {\bf n} \right] \bf{\dot q}^-.\end{equation}

We can add friction at the contact.  Rigid bodies will almost always
experience contact at only a point, so we typically ignore torsional
friction <elib>Featherstone07</elib>, and model only tangential friction
by extending ${\bf n}$ to a matrix

$$\bf J = \begin{bmatrix} {\bf n}\\
{\bf t}_1\\ {\bf t}_2 \end{bmatrix},$$

to capture the gradient of the
contact location tangent to the contact surface. Then $\bf \lambda$ becomes
a Cartesian vector representing the contact impulse, and (for infinite
friction) the post-impact velocity condition becomes ${\bf J}\bf{\dot q}^+
= \text{diag}(-e, 0, 0) {\bf J}\bf{\dot q}^-,$ resulting in the equations:

\begin{equation}\bf{\dot q}^+ = \left[ \bf I - \bf M^{-1} \bf J^T
\text{diag}(1+e, 1, 1) \left[\bf J \bf M^{-1} \bf J^T \right]^\# \bf J
\right]\bf{\dot q}^-.\end{equation}

If $\bf \lambda$ is restricted to a
friction cone, as in Coulomb friction, in general we have to solve an
optimization to solve for $\bf\dot q^+$ subject to the inequality
constraints.

# A spinning ball bouncing on the ground.

Imagine a ball (a hollow-sphere) in the plane with mass $m$, radius
$r$.  The configuration is given by $q = \begin{bmatrix} x, z, \theta
\end{bmatrix}^T.$  The equations of motion are

$$\bf M(\bf q)\bf\ddot q =
\begin{bmatrix} m & 0 & 0 \\ 0 & m & 0 \\ 0 & 0 & \frac{2}{3}mr^2
\end{bmatrix} \bf\ddot q = \begin{bmatrix} 0 \\ -g \\ 0 \end{bmatrix} +
\begin{bmatrix} 0 & 1 \\ 1 & 0 \\ 0 & r \end{bmatrix} \begin{bmatrix}
\lambda_z \\ \lambda_x \end{bmatrix} = \tau_g + \bf J^T {\bf \lambda},$$

where ${\bf \lambda}$ are the contact forces (which are zero except
during the impulsive collision).  Given a coefficient of restitution
$e$ and a collision with a horizontal ground, the post-impact
velocities are:

$$\bf{\dot q}^+ = \begin{bmatrix} \frac{3}{5} & 0 &
-\frac{2}{5} r \\ 0 & - e & 0 \\ -\frac{3}{5r} & 0 &
\frac{2}{5}\end{bmatrix}\bf{\dot q}^-.$$


<!-- Derivation is here: https://www.wolframcloud.com/obj/russt/Published/BouncingBallWithSpin.nb-->

</example>

</subsubsection>

# Putting it all together

 We can put the bilateral constraint equations and the impulsive
collision equations together to implement a hybrid model for unilateral
constraints of the form. Let us generalize \begin{equation}\bf phi(\bf q)
\ge 0,\end{equation} to be the vector of all pairwise (signed) distance
functions between rigid bodies; this vector describes all possible
contacts. At every moment, some subset of the contacts are active, and
can be treated as a bilateral constraint ($\bf phi_i=0$). The guards of
the hybrid model are when an inactive constraint becomes active
($\bf phi_i>0 \rightarrow \bf phi_i=0$), or when an active constraint
becomes inactive ($\bf \lambda_i>0 \rightarrow \bf \lambda_i=0$ and
$\dot\phi_i > 0$). Note that collision events will (almost always) only
result in new active constraints when $e=0$, e.g. we have pure
inelastic collisions, because elastic collisions will rarely permit
sustained contact.

If the contact involves Coulomb friction, then the transitions
between sticking and sliding can be modeled as additional hybrid
guards.

</subsubsection>

</subsection>

# Time-stepping Approximations for Rigid Contact

 So far we have seen two different approaches to enforcing the
inequality constraints of contact, $\phi(\bf q) \ge 0$ and the friction
cone.  First we introduced compliant contact which effectively enforces
non-penetration with a stiff spring.  Then we discussed event detection as
a means to switch  between different models which treat the active
constraints as equality constraints.  But, perhaps surprisingly, one of
the most popular and scalable approaches is something different: it
involves formulating a mathematical program that can solve the inequality
constraints directly on each time step of the simulation.  These
algorithms may be more expensive to compute on each time step, but they
allow for stable simulations with potentially much larger and more
consistent time steps.

# Complementarity formulations

What class of mathematical program due we need to simulate contact?
The discrete nature of contact suggests that we might need some form of
combinatorial optimization.  Indeed, the most common transcription is to
a Linear Complementarity Problem (LCP) <elib>Murty88</elib>, as
introduced by <elib>Stewart96</elib> and <elib>Anitescu97</elib>.  An
LCP is typically written as

\begin{align*} \text{find}_{\bf w,\bf z} \quad \text{subject to}
\quad & \bf w = \bf q + \bf M \bf z, \\ & \bf w \ge 0, \bf z \ge 0, \bf w^T\bf z =
0.\end{align*}

They are directly related to the optimality conditions of
a (potentially non-convex) quadratic program  and
<elib>Anitescu97</elib>> showed that the LCP's generated by our
rigid-body contact problems can be solved by Lemke's algorithm.  As
short-hand we will write these complementarity constraints as

$$\text{find}_{\bf z}\quad \text{subject to} \quad 0 \le (\bf q + \bf M\bf z) \perp \bf z \ge
0.$$

Rather than dive into the full transcription, which has many terms
and can be relatively difficult to parse, let me start with two very
simple examples.

# Time-stepping LCP: Normal force

Consider our favorite simple mass being actuated by a horizontal
force (with the double integrator dynamics), but this time we will add
a wall that will prevent the cart position from taking negative
values: our non-penetration constraint is $q \ge 0$.  Physically, this
constraint is implemented by a normal (horizontal) force, $f$,
yielding the equations of motion:

$$m\ddot{q} = u + f.$$

<figure><img width=40% src="figures/lcp_brick_normal_force.jpg"> 
</figure>

$f$ is defined as the force required to enforce the non-penetration
constraint; certainly the following are true: $f \ge 0$ and $q \cdot
f = 0$.  $q \cdot f = 0$ is the "complementarity constraint", and you
can read it here as "either q is zero or force is zero" (or both); it
is our "no force at a distance" constraint, and it is clearly
non-convex.  It turns out that satisfying these constraints, plus $q
\ge 0$, is sufficient to fully define $f$.

To define the LCP, we first discretize time, by approximating
\begin{gather*}q[n+1] = q[n] + h v[n+1], \\ v[n+1] = v[n] +
\frac{h}{m}(u[n] + f[n]).\end{gather*}  This is almost the standard
Euler approximation, but note the use of $v[n+1]$ in the right-hand
side of the first equation -- this is actually a <a
href="https://en.wikipedia.org/wiki/Semi-implicit_Euler_method">semi-implicit
Euler approximation</a>, and this choice is essential in the
derivation.

Given $h, q[n], v[n],$ and $u[n]$, we can solve for $f[n]$ and
$q[n+1]$ simultaneously, by solving the following LCP: \begin{gather*}
q[n+1] = \left[ q[n] + h v[n] + \frac{h^2}{m} u[n] \right] +
\frac{h^2}{m} f[n] \\ q[n+1] \ge 0, \quad f[n] \ge 0, \quad
q[n+1]\cdot f[n] = 0. \end{gather*}  Take a moment and convince
yourself that this fits into the LCP prescription given above.

<figure><img width=40%
src="figures/lcp_brick_sol_vectors.jpg"></figure>

Note: Please don't confuse this visualization with the more common
visualization of the solution space for an LCP (in two or more
dimensions) in terms of "complementary cones"<elib>Murty88</elib>.

Perhaps it's also helpful to plot the solution, $q[n+1], f[n]$ as a function of $q[n], v[n]$.  I've done it here for $m=1, h=0.1, u[n]=0$:
<figure>
<!--            <iframe id="igraph" scrolling="no" style="border:none;"
seamless="seamless" src="data/lcp_cart.html" height="250"
width="100%"></iframe> -->
</figure>

</example>

In the (time-stepping, "impulse-velocity") LCP formulation, we write
the contact problem in it's combinatorial form.  In the simple example
above, the complementarity constraints force any solution to lie on
<i>either</i> the positive x-axis ($f[n] \ge 0$) <i>or</i> the positive
y-axis ($q[n+1] \ge 0$).  The equality constraint is simply a line that
will intersect with this constraint manifold at the solution.  But in
this frictionless case, it is important to realize that these are simply
the optimality conditions of a convex optimization problem: the
discrete-time version of Gauss's principle that we used above.  Using
$\bf v'$ as shorthand for $\bf v[n+1]$, and replacing $\bf{\ddot q} =
\frac{\bf v' - \bf v}{h}$ in Eq \eqref{eq:least_constraint} and scaling the
objective by $h^2$ we have: \begin{align*} \min_{\bf v'} \quad &
\frac{1}{2} \left(\bf v' - \bf v - h\bf M^{-1}\bf\tau\right)^T \bf M \left(\bf v' -
\bf v - h\bf M^{-1}\bf\tau\right) \\ \text{subject to} \quad & \frac{1}{h}\phi(\bf q') =
\frac{1}{h} \phi(\bf q + h\bf v') \approx \frac{1}{h}\phi(\bf q) + {\bf n}\bf v'
\ge 0. \end{align*}  The objective is even cleaner/more intuitive if we
denote the next step velocity that would have occurred before the
contact impulse is applied as $\bf v^- = \bf v + h\bf M^{-1}\bf\tau$:
\begin{align*} \min_{\bf v'} \quad & \frac{1}{2} \left(\bf v' -
\bf v^-\right)^T \bf M \left(\bf v' - \bf v^-\right) \\ \text{subject to} \quad &
\frac{1}{h}\phi(\bf q') \ge 0. \end{align*} The LCP for the frictionless
contact dynamics is precisely the optimality conditions of this convex
(because $\bf M \succeq 0$) quadratic program, and once again the contact
impulse, ${\bf f} \ge 0$, plays the role of the Lagrange multiplier
(with units $N\cdot s$).

So why do we talk so much about LCPs instead of QPs?  Well LCPs can
also represent a wider class of problems, which is what we arrive at
with the standard transcription of Coulomb friction.  In the limit of
infinite friction, then we could add an additional constraint that the
tangential velocity at each contact point was equal to zero (but these
equation may not always have a feasible solution).  Once we admit limits on the magnitude of the friction forces, the non-convexity of the disjunctive form rear's it's ugly head.

# Time-stepping LCP: Coulomb Friction

We can use LCP to find a feasible solution with Coulomb
friction, but it requires some gymnastics with slack variables to make
it work.  For this case in particular, I believe a very simple
example is best.  Let's take our brick and remove the wall and the
wheels (so we now have friction with the ground).  

<figure><img width=40% src="figures/lcp_brick_friction.jpg"> 
</figure>

The dynamics are the same as our previous example, 

$$m\ddot{q} = u+ f,$$

but this time I'm using $f$ for the friction force which is
inside the friction cone if $\dot{q} = 0$ and on the boundary of the
friction cone if $\dot{q} \ne 0;$ this is known as the principle of
maximum dissipation<elib>Stewart96</elib>.  Here the magnitude of the
normal force is always $mg$, so we have $|f| \le \mu mg,$ where $\mu$
is the coefficient of friction.  And we will use the same
semi-implicit Euler approximation to cast this into discrete time.

Now, to write the friction constraints as an LCP, we must introduce
some slack variables.  First, we'll break the force into a positive
component and a negative component: $f[n] = f^+ - f^-.$  And we will
introduce one more variable $v_{abs}$ which will be non-zero if the
velocity is non-zero (in either direction).  Now we can write the LCP:
\begin{align*} \text{find}_{v_{abs}, f^+, f^-} \quad \text{subject to} && \\ 0 \le
v_{abs} + v[n+1] \quad &\perp& f^+ \ge 0, \\ 0 \le v_{abs} - v[n+1]
\quad &\perp& f^- \ge 0, \\ 0 \le \mu mg - f^+ - f^- \quad &\perp&
v_{abs} \ge 0,\end{align*} where each instance of $v[n+1]$ we write
out with

$$v[n+1] = v[n] + \frac{h}{m}(u + f^+ - f^-).$$

It's enough
to make your head spin!  But if you work it out, all of the
constraints we want are there.  For example, for $v[n+1] > 0$, then we
must have $f^+=0$, $v_{abs} = v[n+1]$, and $f^- = \mu mg$.  When
$v[n+1] = 0$, we can have $v_{abs} = 0$, $f^+ - f^- \le \mu mg$, and
those forces must add up to make $v[n+1] = 0$.

</example>

We can put it all together and write an LCP with both normal forces
and friction forces, related by Coulomb friction (using a polyhedral
approximation of the friction cone in 3d)<elib>Stewart96</elib>.
Although the solution to any LCP <a
href="https://en.wikipedia.org/wiki/Linear_complementarity_problem">can
also be represented as the solution to a QP</a>, the QP for this problem
is non-convex.

</subsubsection>

# Anitescu's convex formulation

In <elib>Anitescu06</elib>, Anitescu described the natural convex
formulation of this problem by dropping the maximum dissipation
inequalities and allowing any force inside the friction cone.  Consider
the following optimization: \begin{align*} \min_{\bf v'} \quad &
\frac{1}{2} \left( \bf v' - \bf v^-\right)^T \bf M \left(\bf v' - \bf v^-\right)
\\ \text{subject to} \quad & \frac{1}{h}\phi(\bf q') + \mu {\bf d}_i \bf v' \ge 0,
\qquad \forall i \in 1,...,m \end{align*} where ${\bf d}_i, \forall i\in
1,...,m$ are a set of tangent row vectors in joint coordinates
parameterizing the polyhedral approximation of the friction cone (as in
<elib>Stewart96</elib>).  Importantly, for each ${\bf d}_i$ we must also
have $-{\bf d}_i$ in the set.  By writing the Lagrangian,

$$L(\bf v', {\bf
\beta}) = \frac{1}{2}(\bf v' - \bf v^-)^T\bf M(\bf v' - \bf v^-) - \sum_i \beta_i
\left(\frac{1}{h}\phi(\bf q) + ({\bf n} + \mu {\bf d})\bf v'\right),$$

and checking the stationarity condition:

$$\frac{\partial L}{\partial \bf v'}^T = \bf M(\bf v' - \bf v) - h\bf\tau -
\sum_i \beta_i ({\bf n} + \mu{\bf d}_i)^T = 0,$$

we can see that the
Lagrange multiplier, $\beta_i \ge 0$, associated with the $i$th
constraint is the magnitude of the impulse (with units $N \cdot s$) in
the direction ${\bf n} - \mu {\bf d}_i$, where ${\bf n} = \frac{\partial \phi}{\partial \bf q}$ is the contact normal; the sum of the forces is an
affine combination of these extreme rays of the polyhedral friction
cone.  As written, the optimization is a QP.  

But be careful!  Although the primal solution was convex, and the
dual problems are always convex, the objective here can be positive
semi-definite.  This isn't a mistake -- <elib>Anitescu06</elib>
describes a few simple examples where the solution to $\bf v'$ is unique,
but the impulses that produce it are not (think of a table with four
legs).

When the tangential velocity is zero, this constraint is tight; for
sliding contact the relaxation effectively causes the contact to
"hydroplane" up out of contact, because $\phi(\bf q') \ge h\mu {\bf d}_i\bf v'.$  It seems like a quite reasonable
approximation to me, especially for small $h$!

Let's continue on and write the dual program.  To make the notation a
little better, the us stack the Jacobian vectors into $\bf J_\beta$, such
that the $i$th row is ${\bf n} + \mu{\bf d_i}$, so that we have
$$\frac{\partial L}{\partial \bf v'}^T = \bf M(\bf v' - \bf v) - h\bf\tau - \bf J_\beta^T \beta = 0.$$
Substituting this back into the Lagrangian give us the dual program:
$$\min_{\beta \ge 0} \frac{1}{2} \beta^T \bf J_\beta \bf M^{-1} \bf J_\beta^T
\beta +  \frac{1}{h}\phi(\bf q) \sum_i \beta_i.$$

One final note: I've written just one contact point here to simplify
the notation, but of course we repeat the constraint(s) for each contact
point; <elib>Anitescu06</elib> studies the typical case of only
including potential contact pairs with $\phi(\bf q) \le \epsilon$, for
small $\epsilon \ge 0$.  

</subsubsection>

# Todorov's convex formulation

According to <elib>Todorov14</elib>, the popular <a
href="http://www.mujoco.org/">MuJoCo simulator</a> uses a slight
variation of this convex relaxation, with the optimization:
\begin{align*} \min_\lambda \quad & \frac{1}{2} \lambda^T \bf J \bf M^{-1}
\bf J^T \lambda + \frac{1}{h}\lambda^T \left( \bf J \bf v^- - \dot\bf x^d
\right), \\ \text{subject to} \quad & \lambda \in \mathcal{FC}(\bf q),\end{align*}
where $\bf\dot x^d$ is a "desired contact velocity", and
$\mathcal{FC}(\bf q)$ describes the friction cone.  The friction cone
constraint looks new but is not; observe that $\bf J^T \lambda =
\bf J_\beta^T \beta$, where $\beta \ge 0$ is a clever parameterization of
the friction cone in the polyhedral case.  The bigger difference is in
the linear term: <elib>Todorov14</elib> proposed $\bf\dot x^d = \bf J \bf v -
h\mathcal{B} \bf J \bf v - h \mathcal{K} [\phi(\bf q), 0, 0]^T,$ with
$\mathcal{B}$ and $\mathcal{K}$ stabilization gains that are chosen to
yield a critically damped response. 

How should we interpret this?  If you expand the linear terms, you
will see that this is almost exactly the dual form we get from the
position equality constraints formulation, including the Baumgarte
stabilization.  <!--One term is missing: the $\dot\bf J\bf v$ term, but
<elib>Todorov14</elib> avoids this by saying that they consider
constraints of the form $\bf J\bf v = 0 (+ \text{stabilization});$ I think
that's a clever way of saying you don't want to bother computing the
discrete-time equivalent of $\dot\bf J\bf v$. --> It's interesting to think
about the implications of this formulation -- like the Anitescu
relaxation, it is possible to get some "force at a distance" because we
have not in any way expressed that $\lambda = 0$ when $\phi(\bf q) > 0.$
In Anitescu, it happened in a velocity-dependent boundary layer; here it
could happen if you are "coming in hot" -- moving towards contact with
enough relative velocity that the stabilization terms want to slow you
down.

In dual form, it is natural to consider the full conic description of
the friction cone: 

$$\mathcal{FC}(\bf q) = \left\{{\bf \lambda} = [f_n,
f_{t1}, f_{t2}]^T \middle| f_n \ge 0, \sqrt{f^2_{t1}+f^2_{t2}} \le \mu
f_n \right\}.$$

The resulting dual optimization is has a quadratic
objective and second-order cone constraints (SOCP).

There are a number of other important relaxations that happen in
MuJoCo.  To address the positive indefiniteness in the objective,
<elib>Todorov14</elib> relaxes the dual objective by adding a small term
to the diagonal.  This guarantees convexity, but the convex optimization
can still be poorly conditioned.  The stabilization terms in the
objective are another form of relaxation, and <elib>Todorov14</elib>
also implements adding additional stabilization to the inertial matrix,
called the "implicit damping inertia".  These relaxations can
dramatically improve simulation speed, both by making each convex
optimization faster to solve, and by allowing the simulator to take
larger integration time steps.  MuJoCo boasts a special-purpose solver
that can simulate complex scenes at seriously impressive speeds -- often
orders of magnitude faster than real time.  But these relaxations can
also be abused -- it's unfortunately quite common to see physically
absurd MuJoCo simulations, especially when researchers who don't care
much about the physics start changing simulation parameters to optimize
their learning curves!

</subsubsection>

<todo>

# Relaxed complementarity-free formulation: Normal force
@todo

What equations do we get if we simulate the simple cart colliding
with a wall example that we used above?

<figure><img width=40% src="figures/lcp_brick_normal_force.jpg"> 
</figure>

Following <elib>Todorov14</elib>, the first relaxation we find is
the addition of "implicit damping inertia", which effectively adds
some inertia and some damping, parameterized by the regularization
term $b$, to the explicit form of the equations.  As a result our
discrete-time cart dynamics are given by

$$(m+b)\frac{v[n+1] - v[n]}{h} = u[n] + f[n] - bv[n].$$

What remains is to compute $f[n]$, and here is where things get
interesting.  Motivated by the idea of "softening the Gauss principle"
(e.g. the principle of least constraint <elib>Udwadia92</elib>) by
replacing the hard constraints with a soft penalty term, we end up
with a convex optimization which, in this scalar frictionless case, reduces to:

$$f[n] = \argmin_{f \ge 0}  \frac{1}{2} f^2 \frac{1 + \epsilon}{m+b} + f(v^- - v^*),$$

where $v^- =
v[n] + \frac{h}{m + b}(u[n] - bv[n])$ is the next-step velocity before
the contact impulse.  $v^*$ is another regularization term: the
"desired contact velocity" which is implemented like a by Baumgarte
stabilization:

$$v^* = v[n] - 2h \frac{1 + \epsilon}{\kappa} v[n] - h\frac{1+\epsilon}{\kappa^2} q[n].$$

This introduced two additional
relaxation parameters; MuJoCo calls $\epsilon$ the "impulse
regularization scaling" and $\kappa$ the "error-reduction time
constant".
</example>
</todo>

# Beyond Point Contact

Coming soon... Box-on-plane example.  Multi-contact.

Also a single point force cannot capture effects like torsional
friction, and performs badly in some very simple cases (imaging a box
sitting on a table).  As a result, many of the most effective algorithms
for contact restrict themselves to very limited/simplified geometry.
For instance, one can place "point contacts" (zero-radius spheres) in
the four corners of a robot's foot; in this case adding forces at
exactly those four points as they penetrate some other body/mesh can
give more consistent contact force locations/directions. A surprising
number of simulators, especially for legged robots, do this.

In practice, most collision detection algorithms will return a list
of "collision point pairs" given the list of bodies (with one point pair
per pair of bodies in collision, or more if they are using the
aforementioned "multi-contact" heuristics), and our contact force model
simply attaches springs between these pairs. Given a point-pair, $p_A$
and $p_B$, both in world coordinates, ...

Hydroelastic model in drake<elib>Elandt19</elib>. 

[mit.edu/multibody](http://underactuated.mit.edu/multibody.html)
