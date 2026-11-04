# Source this file from Bash to load the project's shortcuts.
_amp_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# MUJOCO_ROOT can override the default MuJoCo installation directory.
alias g1="\"${MUJOCO_ROOT:-$HOME/mujoco-install}/bin/simulate\" \"$_amp_repo_root/unitree_ros/robots/g1_description/g1_29dof.xml\""

unset _amp_repo_root
