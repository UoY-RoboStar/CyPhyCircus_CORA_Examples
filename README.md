This archive contains the files for the models accompanying the paper "Model checking for a hybrid process algebra". Each of the models is in its own folder: `SimpleRanger`, `Ranger` and `Navigation`. The CORA models have been tested in CORA 2024.4.1 using MATLAB R2024a. The CyPhyCircus models are written in LaTeX syntax for CZT.

# `SimpleRanger`

This directory contains the folllowing files:

* `ranger_simple.zed` - the CyPhyCircus model for the simple ranger and its controller shown in the paper
* `ranger_simple-1-inline_sets.zed` - the CyPhyCircus model after the inlining of channel sets and namesets during preprocessing (there are replicated operators or interleavings to be eliminated)
* `ranger_simple-2-rewrite-inputs.zed` - the CyPhyCircus model after input communications are rewritten to introduce a variable block during preprocessing (there are no parametrised or generic processes or actions to instantiate)
* `ranger_simple-3-lift_variable_blocks.zed` - the CyPhyCircus model after variables from variable blocks have been lifted into the process state during preprocessing
* `ranger_simple-4-expand_extchoice_actions.zed` - the CyPhyCircus model after actions in external choices have been expanded during preprocessing
* `ranger_simple-5-distribute_extchoice_seqcomps.zed` - the CyPhyCircus model after sequential compositions have been distributed leftwards into the branches of an external choice during preprocessing
* `ranger_simple-6-convert_extchoice_guards_to_conditionals.zed` - the CyPhyCircus model after external choices with guards have been converted to conditionals whose branches are external choices during preprocessing
* `ranger_simple-7-name_actions.zed` - the CyPhyCircus model after the actions have been split into individual named actions during preprocessing, resulting in the normalised model to be translated
* `ranger_simple_structure.md` - notes on the calculation of the synchronisation labels and structure of the automata used during hand-translation of the model
* `ranger_simple_translation.m` - the CORA model resulting from the translation, along with its checks
* `roboworld_2d_toolkit.zed` - additional definitions depended on by the CyPhyCircus models

# `Ranger`

This directory contains the folllowing files:

* `ranger.zed` - the CyPhyCircus model for the full ranger discussed in the paper
* `ranger-1-expand_parallelism.zed` - the CyPhyCircus model after replacing interleavings with generalised parallel operators during preprocessing (there are replicated operators to be eliminated)
* `ranger-2-inline_sets.zed` - the CyPhyCircus model after the inlining of channel sets and namesets during preprocessing
* `ranger-3-rewrite-inputs.zed` - the CyPhyCircus model after input communications are rewritten to introduce a variable block during preprocessing (there are no parametrised or generic processes or actions to instantiate)
* `ranger-4-lift_variable_blocks.zed` - the CyPhyCircus model after variables from variable blocks have been lifted into the process state during preprocessing
* `ranger-5-expand_extchoice_actions.zed` - the CyPhyCircus model after actions in external choices have been expanded during preprocessing
* `ranger-6-distribute_extchoice_seqcomps.zed` - the CyPhyCircus model after sequential compositions have been distributed leftwards into the branches of an external choice during preprocessing
* `ranger-7-convert_extchoice_guards_to_conditionals.zed` - the CyPhyCircus model after external choices with guards have been converted to conditionals whose branches are external choices during preprocessing
* `ranger-8-name_actions.zed` - the CyPhyCircus model after the actions have been split into individual named actions during preprocessing, resulting in the normalised model to be translated
* `ranger_structure.md` - notes on the calculation of the synchronisation labels and structure of the automata used during hand-translation of the model
* `ranger_translation.m` - the CORA model resulting from the translation, along with its checks
* `roboworld_2d_toolkit.zed` - additional definitions depended on by the CyPhyCircus models

# `Navigation`

This directory contains the folllowing files:

* `navigation.zed` - the CyPhyCircus model for the navigation model with more complex communications discussed in the paper
* `navigation-1-expand_parallelism.zed` - the CyPhyCircus model after replacing interleavings with generalised parallel operators during preprocessing (there are replicated operators to be eliminated)
* `navigation-2-inline_sets.zed` - the CyPhyCircus model after the inlining of channel sets and namesets during preprocessing
* `navigation-3-rewrite-inputs.zed` - the CyPhyCircus model after input communications are rewritten to introduce a variable block during preprocessing (there are no parametrised or generic processes or actions to instantiate)
* `navigation-4-lift_variable_blocks.zed` - the CyPhyCircus model after variables from variable blocks have been lifted into the process state during preprocessing
* `navigation-5-expand_extchoice_actions.zed` - the CyPhyCircus model after actions in external choices have been expanded during preprocessing
* `navigation-6-distribute_extchoice_seqcomps.zed` - the CyPhyCircus model after sequential compositions have been distributed leftwards into the branches of an external choice during preprocessing
* `navigation-7-convert_extchoice_guards_to_conditionals.zed` - the CyPhyCircus model after external choices with guards have been converted to conditionals whose branches are external choices during preprocessing
* `navigation-8-name_actions.zed` - the CyPhyCircus model after the actions have been split into individual named actions during preprocessing, resulting in the normalised model to be translated
* `navigation_structure.md` - notes on the calculation of the synchronisation labels and structure of the automata used during hand-translation of the model
* `navigation_translation.m` - the CORA model resulting from the translation
* `navigation_checks.m` - the checks over the navigation CORA model,  split into a separate file due to their length
* `roboworld_2d_toolkit.zed` - additional definitions depended on by the CyPhyCircus models


