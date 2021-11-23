classdef  Function < casadi.SharedObject & casadi.PrintableCommon
    %FUNCTION Function object A Function instance is a general multiple-input, multiple-
    %
    %
    %output function where each input and output can be a sparse matrix. .
    %
    %For an introduction to this class, see the CasADi user guide. Function is a
    %reference counted and immutable class; copying a class instance is very
    %cheap and its behavior (with some exceptions) is not affected by calling its
    %member functions. Joel Andersson >List of available options
    %
    %+------------------+-----------------+------------------+------------------+
    %|        Id        |      Type       |   Description    |     Used in      |
    %+==================+=================+==================+==================+
    %| ad_weight        | OT_DOUBLE       | Weighting factor | casadi::Function |
    %|                  |                 | for derivative   | Internal         |
    %|                  |                 | calculation.When |                  |
    %|                  |                 | there is an      |                  |
    %|                  |                 | option of either |                  |
    %|                  |                 | using forward or |                  |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | directional      |                  |
    %|                  |                 | derivatives, the |                  |
    %|                  |                 | condition ad_wei |                  |
    %|                  |                 | ght*nf<=(1-ad_we |                  |
    %|                  |                 | ight)*na is used |                  |
    %|                  |                 | where nf and na  |                  |
    %|                  |                 | are estimates of |                  |
    %|                  |                 | the number of    |                  |
    %|                  |                 | forward/reverse  |                  |
    %|                  |                 | mode directional |                  |
    %|                  |                 | derivatives      |                  |
    %|                  |                 | needed. By       |                  |
    %|                  |                 | default,         |                  |
    %|                  |                 | ad_weight is     |                  |
    %|                  |                 | calculated       |                  |
    %|                  |                 | automatically,   |                  |
    %|                  |                 | but this can be  |                  |
    %|                  |                 | overridden by    |                  |
    %|                  |                 | setting this     |                  |
    %|                  |                 | option. In       |                  |
    %|                  |                 | particular, 0    |                  |
    %|                  |                 | means forcing    |                  |
    %|                  |                 | forward mode and |                  |
    %|                  |                 | 1 forcing        |                  |
    %|                  |                 | reverse mode.    |                  |
    %|                  |                 | Leave unset for  |                  |
    %|                  |                 | (class specific) |                  |
    %|                  |                 | heuristics.      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| ad_weight_sp     | OT_DOUBLE       | Weighting factor | casadi::Function |
    %|                  |                 | for sparsity     | Internal         |
    %|                  |                 | pattern          |                  |
    %|                  |                 | calculation calc |                  |
    %|                  |                 | ulation.Override |                  |
    %|                  |                 | s default        |                  |
    %|                  |                 | behavior. Set to |                  |
    %|                  |                 | 0 and 1 to force |                  |
    %|                  |                 | forward and      |                  |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | respectively.    |                  |
    %|                  |                 | Cf. option       |                  |
    %|                  |                 | "ad_weight".     |                  |
    %|                  |                 | When set to -1,  |                  |
    %|                  |                 | sparsity is      |                  |
    %|                  |                 | completely       |                  |
    %|                  |                 | ignored and      |                  |
    %|                  |                 | dense matrices   |                  |
    %|                  |                 | are used.        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| always_inline    | OT_BOOL         | Force inlining.  | casadi::Function |
    %|                  |                 |                  | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| compiler         | OT_STRING       | Just-in-time     | casadi::Function |
    %|                  |                 | compiler plugin  | Internal         |
    %|                  |                 | to be used.      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| custom_jacobian  | OT_FUNCTION     | Override         | casadi::Function |
    %|                  |                 | CasADi's AD. Use | Internal         |
    %|                  |                 | together with    |                  |
    %|                  |                 | 'jac_penalty':   |                  |
    %|                  |                 | 0. Note: Highly  |                  |
    %|                  |                 | experimental.    |                  |
    %|                  |                 | Syntax may break |                  |
    %|                  |                 | often.           |                  |
    %+------------------+-----------------+------------------+------------------+
    %| derivative_of    | OT_FUNCTION     | The function is  | casadi::Function |
    %|                  |                 | a derivative of  | Internal         |
    %|                  |                 | another          |                  |
    %|                  |                 | function. The    |                  |
    %|                  |                 | type of          |                  |
    %|                  |                 | derivative       |                  |
    %|                  |                 | (directional     |                  |
    %|                  |                 | derivative,      |                  |
    %|                  |                 | Jacobian) is     |                  |
    %|                  |                 | inferred from    |                  |
    %|                  |                 | the function     |                  |
    %|                  |                 | name.            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump             | OT_BOOL         | Dump function to | casadi::Function |
    %|                  |                 | file upon first  | Internal         |
    %|                  |                 | evaluation.      |                  |
    %|                  |                 | [false]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_dir         | OT_STRING       | Directory to     | casadi::Function |
    %|                  |                 | dump             | Internal         |
    %|                  |                 | inputs/outputs   |                  |
    %|                  |                 | to. Make sure    |                  |
    %|                  |                 | the directory    |                  |
    %|                  |                 | exists [.]       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_format      | OT_STRING       | Choose file      | casadi::Function |
    %|                  |                 | format to dump   | Internal         |
    %|                  |                 | matrices. See    |                  |
    %|                  |                 | DM.from_file     |                  |
    %|                  |                 | [mtx]            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_in          | OT_BOOL         | Dump numerical   | casadi::Function |
    %|                  |                 | values of inputs | Internal         |
    %|                  |                 | to file          |                  |
    %|                  |                 | (readable with   |                  |
    %|                  |                 | DM.from_file )   |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_out         | OT_BOOL         | Dump numerical   | casadi::Function |
    %|                  |                 | values of        | Internal         |
    %|                  |                 | outputs to file  |                  |
    %|                  |                 | (readable with   |                  |
    %|                  |                 | DM.from_file )   |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_fd        | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation by   |                  |
    %|                  |                 | finite           |                  |
    %|                  |                 | differencing.    |                  |
    %|                  |                 | [default:        |                  |
    %|                  |                 | false]]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_forward   | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | Jacobian-times-  |                  |
    %|                  |                 | vector products  |                  |
    %|                  |                 | - typically      |                  |
    %|                  |                 | using forward    |                  |
    %|                  |                 | mode AD - if     |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_jacobian  | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | Jacobians of all |                  |
    %|                  |                 | differentiable   |                  |
    %|                  |                 | outputs with     |                  |
    %|                  |                 | respect to all   |                  |
    %|                  |                 | differentiable   |                  |
    %|                  |                 | inputs - if      |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_reverse   | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | transposed       |                  |
    %|                  |                 | Jacobian-times-  |                  |
    %|                  |                 | vector products  |                  |
    %|                  |                 | - typically      |                  |
    %|                  |                 | using reverse    |                  |
    %|                  |                 | mode AD - if     |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| fd_method        | OT_STRING       | Method for       | casadi::Function |
    %|                  |                 | finite           | Internal         |
    %|                  |                 | differencing     |                  |
    %|                  |                 | [default         |                  |
    %|                  |                 | 'central']       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| fd_options       | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to the    | Internal         |
    %|                  |                 | finite           |                  |
    %|                  |                 | difference       |                  |
    %|                  |                 | instance         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| forward_options  | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to a      | Internal         |
    %|                  |                 | forward mode     |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| gather_stats     | OT_BOOL         | Deprecated       | casadi::Function |
    %|                  |                 | option           | Internal         |
    %|                  |                 | (ignored):       |                  |
    %|                  |                 | Statistics are   |                  |
    %|                  |                 | now always       |                  |
    %|                  |                 | collected.       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| input_scheme     | OT_STRINGVECTOR | Deprecated       | casadi::Function |
    %|                  |                 | option (ignored) | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| inputs_check     | OT_BOOL         | Throw exceptions | casadi::Function |
    %|                  |                 | when the         | Internal         |
    %|                  |                 | numerical values |                  |
    %|                  |                 | of the inputs    |                  |
    %|                  |                 | don't make sense |                  |
    %+------------------+-----------------+------------------+------------------+
    %| is_diff_in       | OT_BOOLVECTOR   | Indicate for     | casadi::Function |
    %|                  |                 | each input if it | Internal         |
    %|                  |                 | should be        |                  |
    %|                  |                 | differentiable.  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| is_diff_out      | OT_BOOLVECTOR   | Indicate for     | casadi::Function |
    %|                  |                 | each output if   | Internal         |
    %|                  |                 | it should be     |                  |
    %|                  |                 | differentiable.  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jac_penalty      | OT_DOUBLE       | When requested   | casadi::Function |
    %|                  |                 | for a number of  | Internal         |
    %|                  |                 | forward/reverse  |                  |
    %|                  |                 | directions, it   |                  |
    %|                  |                 | may be cheaper   |                  |
    %|                  |                 | to compute first |                  |
    %|                  |                 | the full         |                  |
    %|                  |                 | jacobian and     |                  |
    %|                  |                 | then multiply    |                  |
    %|                  |                 | with seeds,      |                  |
    %|                  |                 | rather than      |                  |
    %|                  |                 | obtain the       |                  |
    %|                  |                 | requested        |                  |
    %|                  |                 | directions in a  |                  |
    %|                  |                 | straightforward  |                  |
    %|                  |                 | manner. Casadi   |                  |
    %|                  |                 | uses a heuristic |                  |
    %|                  |                 | to decide which  |                  |
    %|                  |                 | is cheaper. A    |                  |
    %|                  |                 | high value of    |                  |
    %|                  |                 | 'jac_penalty'    |                  |
    %|                  |                 | makes it less    |                  |
    %|                  |                 | likely for the   |                  |
    %|                  |                 | heurstic to      |                  |
    %|                  |                 | chose the full   |                  |
    %|                  |                 | Jacobian         |                  |
    %|                  |                 | strategy. The    |                  |
    %|                  |                 | special value -1 |                  |
    %|                  |                 | indicates never  |                  |
    %|                  |                 | to use the full  |                  |
    %|                  |                 | Jacobian         |                  |
    %|                  |                 | strategy         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit              | OT_BOOL         | Use just-in-time | casadi::Function |
    %|                  |                 | compiler to      | Internal         |
    %|                  |                 | speed up the     |                  |
    %|                  |                 | evaluation       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_cleanup      | OT_BOOL         | Cleanup up the   | casadi::Function |
    %|                  |                 | temporary source | Internal         |
    %|                  |                 | file that jit    |                  |
    %|                  |                 | creates.         |                  |
    %|                  |                 | Default: true    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_name         | OT_STRING       | The file name    | casadi::Function |
    %|                  |                 | used to write    | Internal         |
    %|                  |                 | out code. The    |                  |
    %|                  |                 | actual file      |                  |
    %|                  |                 | names used       |                  |
    %|                  |                 | depend on 'jit_t |                  |
    %|                  |                 | emp_suffix' and  |                  |
    %|                  |                 | include          |                  |
    %|                  |                 | extensions.      |                  |
    %|                  |                 | Default:         |                  |
    %|                  |                 | 'jit_tmp'        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_options      | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to the    | Internal         |
    %|                  |                 | jit compiler.    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_serialize    | OT_STRING       | Specify          | casadi::Function |
    %|                  |                 | behaviour when   | Internal         |
    %|                  |                 | serializing a    |                  |
    %|                  |                 | jitted function: |                  |
    %|                  |                 | SOURCE|link|embe |                  |
    %|                  |                 | d.               |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_temp_suffix  | OT_BOOL         | Use a temporary  | casadi::Function |
    %|                  |                 | (seemingly       | Internal         |
    %|                  |                 | random) filename |                  |
    %|                  |                 | suffix for       |                  |
    %|                  |                 | generated code   |                  |
    %|                  |                 | and libraries.   |                  |
    %|                  |                 | This is desired  |                  |
    %|                  |                 | for thread-      |                  |
    %|                  |                 | safety. This     |                  |
    %|                  |                 | behaviour may    |                  |
    %|                  |                 | defeat caching   |                  |
    %|                  |                 | compiler         |                  |
    %|                  |                 | wrappers.        |                  |
    %|                  |                 | Default: true    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| max_num_dir      | OT_INT          | Specify the      | casadi::Function |
    %|                  |                 | maximum number   | Internal         |
    %|                  |                 | of directions    |                  |
    %|                  |                 | for derivative   |                  |
    %|                  |                 | functions.       |                  |
    %|                  |                 | Overrules the    |                  |
    %|                  |                 | builtin optimize |                  |
    %|                  |                 | d_num_dir.       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| never_inline     | OT_BOOL         | Forbid inlining. | casadi::Function |
    %|                  |                 |                  | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| output_scheme    | OT_STRINGVECTOR | Deprecated       | casadi::Function |
    %|                  |                 | option (ignored) | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| print_in         | OT_BOOL         | Print numerical  | casadi::Function |
    %|                  |                 | values of inputs | Internal         |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_out        | OT_BOOL         | Print numerical  | casadi::Function |
    %|                  |                 | values of        | Internal         |
    %|                  |                 | outputs          |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_time       | OT_BOOL         | print            | casadi::Function |
    %|                  |                 | information      | Internal         |
    %|                  |                 | about execution  |                  |
    %|                  |                 | time. Implies    |                  |
    %|                  |                 | record_time.     |                  |
    %+------------------+-----------------+------------------+------------------+
    %| record_time      | OT_BOOL         | record           | casadi::Function |
    %|                  |                 | information      | Internal         |
    %|                  |                 | about execution  |                  |
    %|                  |                 | time, for        |                  |
    %|                  |                 | retrieval with   |                  |
    %|                  |                 | stats() .        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| regularity_check | OT_BOOL         | Throw exceptions | casadi::Function |
    %|                  |                 | when NaN or Inf  | Internal         |
    %|                  |                 | appears during   |                  |
    %|                  |                 | evaluation       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| reverse_options  | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to a      | Internal         |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| user_data        | OT_VOIDPTR      | A user-defined   | casadi::Function |
    %|                  |                 | field that can   | Internal         |
    %|                  |                 | be used to       |                  |
    %|                  |                 | identify the     |                  |
    %|                  |                 | function or pass |                  |
    %|                  |                 | additional       |                  |
    %|                  |                 | information      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| verbose          | OT_BOOL         | Verbose          | casadi::Function |
    %|                  |                 | evaluation  for  | Internal         |
    %|                  |                 | debugging        |                  |
    %+------------------+-----------------+------------------+------------------+
    %
    %C++ includes: function.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(678, self);
          self.SwigClear();
        end
    end
    function varargout = expand(self,varargin)
    %EXPAND Expand a function to SX.
    %
    %  Function = EXPAND(self)
    %  Function = EXPAND(self, char name, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(679, self, varargin{:});
    end
    function varargout = n_in(self,varargin)
    %N_IN Get the number of function inputs.
    %
    %  int = N_IN(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(680, self, varargin{:});
    end
    function varargout = n_out(self,varargin)
    %N_OUT Get the number of function outputs.
    %
    %  int = N_OUT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(681, self, varargin{:});
    end
    function varargout = size1_in(self,varargin)
    %SIZE1_IN Get input dimension.
    %
    %  int = SIZE1_IN(self, int ind)
    %  int = SIZE1_IN(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(682, self, varargin{:});
    end
    function varargout = size2_in(self,varargin)
    %SIZE2_IN Get input dimension.
    %
    %  int = SIZE2_IN(self, int ind)
    %  int = SIZE2_IN(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(683, self, varargin{:});
    end
    function varargout = size_in(self,varargin)
    %SIZE_IN Get input dimension.
    %
    %  [int,int] = SIZE_IN(self, int ind)
    %  [int,int] = SIZE_IN(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(684, self, varargin{:});
    end
    function varargout = size1_out(self,varargin)
    %SIZE1_OUT Get output dimension.
    %
    %  int = SIZE1_OUT(self, int ind)
    %  int = SIZE1_OUT(self, char oname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(685, self, varargin{:});
    end
    function varargout = size2_out(self,varargin)
    %SIZE2_OUT Get output dimension.
    %
    %  int = SIZE2_OUT(self, int ind)
    %  int = SIZE2_OUT(self, char oname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(686, self, varargin{:});
    end
    function varargout = size_out(self,varargin)
    %SIZE_OUT Get output dimension.
    %
    %  [int,int] = SIZE_OUT(self, int ind)
    %  [int,int] = SIZE_OUT(self, char oname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(687, self, varargin{:});
    end
    function varargout = nnz_in(self,varargin)
    %NNZ_IN Get number of input nonzeros.
    %
    %  int = NNZ_IN(self)
    %  int = NNZ_IN(self, int ind)
    %  int = NNZ_IN(self, char iname)
    %
    %
    %For a particular input or for all of the inputs
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(688, self, varargin{:});
    end
    function varargout = nnz_out(self,varargin)
    %NNZ_OUT Get number of output nonzeros.
    %
    %  int = NNZ_OUT(self)
    %  int = NNZ_OUT(self, int ind)
    %  int = NNZ_OUT(self, char oname)
    %
    %
    %For a particular output or for all of the outputs
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(689, self, varargin{:});
    end
    function varargout = numel_in(self,varargin)
    %NUMEL_IN Get number of input elements.
    %
    %  int = NUMEL_IN(self)
    %  int = NUMEL_IN(self, int ind)
    %  int = NUMEL_IN(self, char iname)
    %
    %
    %For a particular input or for all of the inputs
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(690, self, varargin{:});
    end
    function varargout = numel_out(self,varargin)
    %NUMEL_OUT Get number of output elements.
    %
    %  int = NUMEL_OUT(self)
    %  int = NUMEL_OUT(self, int ind)
    %  int = NUMEL_OUT(self, char oname)
    %
    %
    %For a particular output or for all of the outputs
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(691, self, varargin{:});
    end
    function varargout = name_in(self,varargin)
    %NAME_IN Get input scheme name by index.
    %
    %  {char} = NAME_IN(self)
    %  char = NAME_IN(self, int ind)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  NAME_IN(self, int ind)
    %
    %
    %
    %Get input scheme name by index.
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NAME_IN(self)
    %
    %
    %
    %Get input scheme.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(692, self, varargin{:});
    end
    function varargout = name_out(self,varargin)
    %NAME_OUT Get output scheme name by index.
    %
    %  {char} = NAME_OUT(self)
    %  char = NAME_OUT(self, int ind)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  NAME_OUT(self, int ind)
    %
    %
    %
    %Get output scheme name by index.
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NAME_OUT(self)
    %
    %
    %
    %Get output scheme.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(693, self, varargin{:});
    end
    function varargout = index_in(self,varargin)
    %INDEX_IN Find the index for a string describing a particular entry of an input
    %
    %  int = INDEX_IN(self, char name)
    %
    %scheme.
    %
    %example: schemeEntry("x_opt") -> returns NLPSOL_X if FunctionInternal
    %adheres to SCHEME_NLPINput
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(694, self, varargin{:});
    end
    function varargout = index_out(self,varargin)
    %INDEX_OUT Find the index for a string describing a particular entry of an output
    %
    %  int = INDEX_OUT(self, char name)
    %
    %scheme.
    %
    %example: schemeEntry("x_opt") -> returns NLPSOL_X if FunctionInternal
    %adheres to SCHEME_NLPINput
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(695, self, varargin{:});
    end
    function varargout = default_in(self,varargin)
    %DEFAULT_IN Get default input value.
    %
    %  double = DEFAULT_IN(self, int ind)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(696, self, varargin{:});
    end
    function varargout = max_in(self,varargin)
    %MAX_IN Get largest input value.
    %
    %  double = MAX_IN(self, int ind)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(697, self, varargin{:});
    end
    function varargout = min_in(self,varargin)
    %MIN_IN Get smallest input value.
    %
    %  double = MIN_IN(self, int ind)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(698, self, varargin{:});
    end
    function varargout = sparsity_in(self,varargin)
    %SPARSITY_IN Get sparsity of a given input.
    %
    %  Sparsity = SPARSITY_IN(self, int ind)
    %  Sparsity = SPARSITY_IN(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(699, self, varargin{:});
    end
    function varargout = sparsity_out(self,varargin)
    %SPARSITY_OUT Get sparsity of a given output.
    %
    %  Sparsity = SPARSITY_OUT(self, int ind)
    %  Sparsity = SPARSITY_OUT(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(700, self, varargin{:});
    end
    function varargout = is_diff_in(self,varargin)
    %IS_DIFF_IN Get differentiability of inputs/output.
    %
    %  [bool] = IS_DIFF_IN(self)
    %  bool = IS_DIFF_IN(self, int ind)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(701, self, varargin{:});
    end
    function varargout = is_diff_out(self,varargin)
    %IS_DIFF_OUT Get differentiability of inputs/output.
    %
    %  [bool] = IS_DIFF_OUT(self)
    %  bool = IS_DIFF_OUT(self, int ind)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(702, self, varargin{:});
    end
    function varargout = factory(self,varargin)
    %FACTORY 
    %
    %  Function = FACTORY(self, char name, {char} s_in, {char} s_out, struct:{char} aux, struct opts)
    %
    %
      [varargout{1:nargout}] = casadiMEX(703, self, varargin{:});
    end
    function varargout = oracle(self,varargin)
    %ORACLE Get oracle.
    %
    %  Function = ORACLE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(704, self, varargin{:});
    end
    function varargout = wrap(self,varargin)
    %WRAP Wrap in an Function instance consisting of only one MX call.
    %
    %  Function = WRAP(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(705, self, varargin{:});
    end
    function varargout = wrap_as_needed(self,varargin)
    %WRAP_AS_NEEDED Wrap in a Function with options.
    %
    %  Function = WRAP_AS_NEEDED(self, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(706, self, varargin{:});
    end
    function varargout = which_depends(self,varargin)
    %WHICH_DEPENDS Which variables enter with some order.
    %
    %  [bool] = WHICH_DEPENDS(self, char s_in, {char} s_out, int order, bool tr)
    %
    %
    %Parameters:
    %-----------
    %
    %order:  Only 1 (linear) and 2 (nonlinear) allowed
    %
    %tr:  Flip the relationship. Return which expressions contain the variables
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(707, self, varargin{:});
    end
    function varargout = print_dimensions(self,varargin)
    %PRINT_DIMENSIONS Print dimensions of inputs and outputs.
    %
    %  std::ostream & = PRINT_DIMENSIONS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(708, self, varargin{:});
    end
    function varargout = print_options(self,varargin)
    %PRINT_OPTIONS Print options to a stream.
    %
    %  std::ostream & = PRINT_OPTIONS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(709, self, varargin{:});
    end
    function varargout = print_option(self,varargin)
    %PRINT_OPTION Print all information there is to know about a certain option.
    %
    %  std::ostream & = PRINT_OPTION(self, char name)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(710, self, varargin{:});
    end
    function varargout = uses_output(self,varargin)
    %USES_OUTPUT Do the derivative functions need nondifferentiated outputs?
    %
    %  bool = USES_OUTPUT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(711, self, varargin{:});
    end
    function varargout = jacobian_old(self,varargin)
    %JACOBIAN_OLD Generate a Jacobian function of output oind with respect to input iind.
    %
    %  Function = JACOBIAN_OLD(self, int iind, int oind)
    %
    %
    %Parameters:
    %-----------
    %
    %iind:  The index of the input
    %
    %oind:  The index of the output Legacy function: To be deprecated in a future
    %version of CasADi. Exists only for compatibility with Function::jacobian
    %pre-CasADi 3.2
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(712, self, varargin{:});
    end
    function varargout = hessian_old(self,varargin)
    %HESSIAN_OLD Generate a Hessian function of output oind with respect to input iind.
    %
    %  Function = HESSIAN_OLD(self, int iind, int oind)
    %
    %
    %Parameters:
    %-----------
    %
    %iind:  The index of the input
    %
    %oind:  The index of the output Legacy function: To be deprecated in a future
    %version of CasADi. Exists only for compatibility with Function::hessian pre-
    %CasADi 3.2
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(713, self, varargin{:});
    end
    function varargout = jacobian(self,varargin)
    %JACOBIAN Generate a Jacobian function of all the inputs elements with respect to all
    %
    %  Function = JACOBIAN(self)
    %
    %the output elements).
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(714, self, varargin{:});
    end
    function varargout = jac(self,varargin)
    %JAC Calculate all Jacobian blocks Generates a function that takes all non-
    %
    %  Function = JAC(self)
    %
    %differentiated inputs and outputs and calculates all Jacobian blocks. Inputs
    %that are not needed by the routine are all-zero sparse matrices with the
    %correct dimensions. Output blocks that are not calculated, e.g. if the
    %corresponding input or output is marked non-differentiated are also all-zero
    %sparse. The Jacobian blocks are sorted starting by all the blocks for the
    %first output, then all the blocks for the second output and so on. E.g.
    %f:(x,y)->(r,s) results in the function jac_f:(x,y,r,s)->(dr_dx, dr_dy,
    %ds_dx, ds_dy) This function is cached.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(715, self, varargin{:});
    end
    function varargout = call(self,varargin)
    %CALL Generate a Jacobian function of output oind with respect to input iind.
    %
    %  struct:DM = CALL(self, struct:DM arg, bool always_inline, bool never_inline)
    %  {DM} = CALL(self, {DM} arg, bool always_inline, bool never_inline)
    %  {SX} = CALL(self, {SX} arg, bool always_inline, bool never_inline)
    %  struct:SX = CALL(self, struct:SX arg, bool always_inline, bool never_inline)
    %  struct:MX = CALL(self, struct:MX arg, bool always_inline, bool never_inline)
    %  {MX} = CALL(self, {MX} arg, bool always_inline, bool never_inline)
    %
    %
    %Parameters:
    %-----------
    %
    %iind:  The index of the input
    %
    %oind:  The index of the output Legacy function: To be deprecated in a future
    %version of CasADi. Exists only for compatibility with Function::jacobian
    %pre-CasADi 3.2
    %
    %
    %
    %.......
    %
    %::
    %
    %  CALL(self, {DM} arg, bool always_inline, bool never_inline)
    %
    %
    %
    %Evaluate the function symbolically or numerically.
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CALL(self, struct:DM arg, bool always_inline, bool never_inline)
    %  CALL(self, {SX} arg, bool always_inline, bool never_inline)
    %  CALL(self, struct:SX arg, bool always_inline, bool never_inline)
    %  CALL(self, struct:MX arg, bool always_inline, bool never_inline)
    %  CALL(self, {MX} arg, bool always_inline, bool never_inline)
    %
    %
    %
    %Generate a Jacobian function of output oind with respect to input iind.
    %
    %Parameters:
    %-----------
    %
    %iind:  The index of the input
    %
    %oind:  The index of the output Legacy function: To be deprecated in a future
    %version of CasADi. Exists only for compatibility with Function::jacobian
    %pre-CasADi 3.2
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(716, self, varargin{:});
    end
    function varargout = mapsum(self,varargin)
    %MAPSUM Evaluate symbolically in parallel and sum (matrix graph)
    %
    %  {MX} = MAPSUM(self, {MX} x, char parallelization)
    %
    %
    %Parameters:
    %-----------
    %
    %parallelization:  Type of parallelization used: unroll|serial|openmp
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(717, self, varargin{:});
    end
    function varargout = mapaccum(self,varargin)
    %MAPACCUM Create a mapaccumulated version of this function.
    %
    %  Function = MAPACCUM(self, int N, struct opts)
    %  Function = MAPACCUM(self, char name, int N, struct opts)
    %  Function = MAPACCUM(self, char name, int N, int n_accum, struct opts)
    %  Function = MAPACCUM(self, char name, int n, {char} accum_in, {char} accum_out, struct opts)
    %  Function = MAPACCUM(self, char name, int n, [int] accum_in, [int] accum_out, struct opts)
    %
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for- loop:
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is used to
    %create a tower of function calls, containing unrolled for- loops of length
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but slightly
    %slower at evaluation, than a plain for-loop. The effect is similar to that
    %of a for-loop with a check-pointing instruction after each chunk of
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency here.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(718, self, varargin{:});
    end
    function varargout = fold(self,varargin)
    %FOLD Create a mapaccumulated version of this function.
    %
    %  Function = FOLD(self, int N, struct opts)
    %
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for- loop:
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is used to
    %create a tower of function calls, containing unrolled for- loops of length
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but slightly
    %slower at evaluation, than a plain for-loop. The effect is similar to that
    %of a for-loop with a check-pointing instruction after each chunk of
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency here.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(719, self, varargin{:});
    end
    function varargout = map(self,varargin)
    %MAP Map with reduction A subset of the inputs are non-repeated and a subset of
    %
    %  Function = MAP(self, int n, char parallelization)
    %  Function = MAP(self, int n, [bool] reduce_in, [bool] reduce_out, struct opts)
    %  Function = MAP(self, int n, char parallelization, int max_num_threads)
    %  Function = MAP(self, char name, char parallelization, int n, {char} reduce_in, {char} reduce_out, struct opts)
    %  Function = MAP(self, char name, char parallelization, int n, [int] reduce_in, [int] reduce_out, struct opts)
    %
    %the outputs summed up.
    %
    %
    %
    %.......
    %
    %::
    %
    %  MAP(self, int n, char parallelization, int max_num_threads)
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAP(self, int n, char parallelization)
    %
    %
    %
    %Create a mapped version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (a, p) -> ( s )
    %  
    %
    %
    %
    %The the mapped version has the signature:
    %
    %::
    %
    %     F: (A, P) -> (S )
    %  
    %      with
    %          A: horzcat([a0, a1, ..., a_(N-1)])
    %          P: horzcat([p0, p1, ..., p_(N-1)])
    %          S: horzcat([s0, s1, ..., s_(N-1)])
    %      and
    %          s0 <- f(a0, p0)
    %          s1 <- f(a1, p1)
    %          ...
    %          s_(N-1) <- f(a_(N-1), p_(N-1))
    %  
    %
    %
    %
    %Parameters:
    %-----------
    %
    %parallelization:  Type of parallelization used: unroll|serial|openmp
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAP(self, int n, [bool] reduce_in, [bool] reduce_out, struct opts)
    %  MAP(self, char name, char parallelization, int n, {char} reduce_in, {char} reduce_out, struct opts)
    %  MAP(self, char name, char parallelization, int n, [int] reduce_in, [int] reduce_out, struct opts)
    %
    %
    %
    %Map with reduction A subset of the inputs are non-repeated and a subset of
    %the outputs summed up.
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(720, self, varargin{:});
    end
    function varargout = slice(self,varargin)
    %SLICE returns a new function with a selection of inputs/outputs of the original
    %
    %  Function = SLICE(self, char name, [int] order_in, [int] order_out, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(721, self, varargin{:});
    end
    function varargout = forward(self,varargin)
    %FORWARD Get a function that calculates nfwd forward derivatives.
    %
    %  Function = FORWARD(self, int nfwd)
    %
    %
    %Returns a function with n_in + n_out + n_in inputs and nfwd outputs. The
    %first n_in inputs correspond to nondifferentiated inputs. The next n_out
    %inputs correspond to nondifferentiated outputs. and the last n_in inputs
    %correspond to forward seeds, stacked horizontally The n_out outputs
    %correspond to forward sensitivities, stacked horizontally. * (n_in = n_in(),
    %n_out = n_out())
    %
    %The functions returned are cached, meaning that if called multiple timed
    %with the same value, then multiple references to the same function will be
    %returned.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(725, self, varargin{:});
    end
    function varargout = reverse(self,varargin)
    %REVERSE Get a function that calculates nadj adjoint derivatives.
    %
    %  Function = REVERSE(self, int nadj)
    %
    %
    %Returns a function with n_in + n_out + n_out inputs and n_in outputs. The
    %first n_in inputs correspond to nondifferentiated inputs. The next n_out
    %inputs correspond to nondifferentiated outputs. and the last n_out inputs
    %correspond to adjoint seeds, stacked horizontally The n_in outputs
    %correspond to adjoint sensitivities, stacked horizontally. * (n_in = n_in(),
    %n_out = n_out())
    %
    %(n_in = n_in(), n_out = n_out())
    %
    %The functions returned are cached, meaning that if called multiple timed
    %with the same value, then multiple references to the same function will be
    %returned.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(726, self, varargin{:});
    end
    function varargout = sparsity_jac(self,varargin)
    %SPARSITY_JAC Get, if necessary generate, the sparsity of a Jacobian block
    %
    %  Sparsity = SPARSITY_JAC(self, char iind, int oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, int iind, int oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, int iind, char oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, char iind, char oind, bool compact, bool symmetric)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(727, self, varargin{:});
    end
    function varargout = generate(self,varargin)
    %GENERATE Export / Generate C code for the function.
    %
    %  char = GENERATE(self, struct opts)
    %  char = GENERATE(self, char fname, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(728, self, varargin{:});
    end
    function varargout = generate_dependencies(self,varargin)
    %GENERATE_DEPENDENCIES Export / Generate C code for the dependency function.
    %
    %  char = GENERATE_DEPENDENCIES(self, char fname, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(729, self, varargin{:});
    end
    function varargout = generate_in(self,varargin)
    %GENERATE_IN Export an input file that can be passed to generate C code with a main.
    %
    %  {DM} = GENERATE_IN(self, char fname)
    %  GENERATE_IN(self, char fname, {DM} arg)
    %
    %
    %generate_out  convert_in to convert between dict/map and vector
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(730, self, varargin{:});
    end
    function varargout = generate_out(self,varargin)
    %GENERATE_OUT Export an output file that can be checked with generated C code output.
    %
    %  {DM} = GENERATE_OUT(self, char fname)
    %  GENERATE_OUT(self, char fname, {DM} arg)
    %
    %
    %generate_in  convert_out to convert between dict/map and vector
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(731, self, varargin{:});
    end
    function varargout = serialize(self,varargin)
    %SERIALIZE Serialize.
    %
    %  char = SERIALIZE(self, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(732, self, varargin{:});
    end
    function varargout = save(self,varargin)
    %SAVE Export function in specific language.
    %
    %  SAVE(self, char fname, struct opts)
    %
    %
    %Only allowed for (a subset of) SX/MX Functions
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(733, self, varargin{:});
    end
    function varargout = export_code(self,varargin)
    %EXPORT_CODE Export function in specific language.
    %
    %  char = EXPORT_CODE(self, char lang, struct options)
    %  EXPORT_CODE(self, char lang, char fname, struct options)
    %
    %
    %Only allowed for (a subset of) SX/MX Functions
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(734, self, varargin{:});
    end
    function varargout = stats(self,varargin)
    %STATS Get all statistics obtained at the end of the last evaluate call.
    %
    %  struct = STATS(self, int mem)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(735, self, varargin{:});
    end
    function varargout = sx_in(self,varargin)
    %SX_IN Get symbolic primitives equivalent to the input expressions There is no
    %
    %  {SX} = SX_IN(self)
    %  SX = SX_IN(self, int iind)
    %  SX = SX_IN(self, char iname)
    %
    %guarantee that subsequent calls return unique answers.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(736, self, varargin{:});
    end
    function varargout = mx_in(self,varargin)
    %MX_IN Get symbolic primitives equivalent to the input expressions There is no
    %
    %  {MX} = MX_IN(self)
    %  MX = MX_IN(self, int ind)
    %  MX = MX_IN(self, char iname)
    %
    %guarantee that subsequent calls return unique answers.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(737, self, varargin{:});
    end
    function varargout = sx_out(self,varargin)
    %SX_OUT Get symbolic primitives equivalent to the output expressions There is no
    %
    %  {SX} = SX_OUT(self)
    %  SX = SX_OUT(self, int oind)
    %  SX = SX_OUT(self, char oname)
    %
    %guarantee that subsequent calls return unique answers.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(738, self, varargin{:});
    end
    function varargout = mx_out(self,varargin)
    %MX_OUT Get symbolic primitives equivalent to the output expressions There is no
    %
    %  {MX} = MX_OUT(self)
    %  MX = MX_OUT(self, int ind)
    %  MX = MX_OUT(self, char oname)
    %
    %guarantee that subsequent calls return unique answers.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(739, self, varargin{:});
    end
    function varargout = nz_from_in(self,varargin)
    %NZ_FROM_IN Convert from/to flat vector of input/output nonzeros.
    %
    %  [double] = NZ_FROM_IN(self, {DM} arg)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(740, self, varargin{:});
    end
    function varargout = nz_from_out(self,varargin)
    %NZ_FROM_OUT Convert from/to flat vector of input/output nonzeros.
    %
    %  [double] = NZ_FROM_OUT(self, {DM} arg)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(741, self, varargin{:});
    end
    function varargout = nz_to_in(self,varargin)
    %NZ_TO_IN Convert from/to flat vector of input/output nonzeros.
    %
    %  {DM} = NZ_TO_IN(self, [double] arg)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(742, self, varargin{:});
    end
    function varargout = nz_to_out(self,varargin)
    %NZ_TO_OUT Convert from/to flat vector of input/output nonzeros.
    %
    %  {DM} = NZ_TO_OUT(self, [double] arg)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(743, self, varargin{:});
    end
    function varargout = convert_in(self,varargin)
    %CONVERT_IN Convert from/to input/output lists/map.
    %
    %  {DM} = CONVERT_IN(self, struct:DM arg)
    %  struct:DM = CONVERT_IN(self, {DM} arg)
    %  struct:SX = CONVERT_IN(self, {SX} arg)
    %  {SX} = CONVERT_IN(self, struct:SX arg)
    %  {MX} = CONVERT_IN(self, struct:MX arg)
    %  struct:MX = CONVERT_IN(self, {MX} arg)
    %
    %
    %Will raise an error when an unknown key is used or a list has incorrect
    %size. Does not perform sparsity checking.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(744, self, varargin{:});
    end
    function varargout = convert_out(self,varargin)
    %CONVERT_OUT Convert from/to input/output lists/map.
    %
    %  {DM} = CONVERT_OUT(self, struct:DM arg)
    %  struct:DM = CONVERT_OUT(self, {DM} arg)
    %  struct:SX = CONVERT_OUT(self, {SX} arg)
    %  {SX} = CONVERT_OUT(self, struct:SX arg)
    %  {MX} = CONVERT_OUT(self, struct:MX arg)
    %  struct:MX = CONVERT_OUT(self, {MX} arg)
    %
    %
    %Will raise an error when an unknown key is used or a list has incorrect
    %size. Does not perform sparsity checking.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(745, self, varargin{:});
    end
    function varargout = has_free(self,varargin)
    %HAS_FREE Does the function have free variables.
    %
    %  bool = HAS_FREE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(746, self, varargin{:});
    end
    function varargout = get_free(self,varargin)
    %GET_FREE Get free variables as a string.
    %
    %  {char} = GET_FREE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(747, self, varargin{:});
    end
    function varargout = free_sx(self,varargin)
    %FREE_SX Get all the free variables of the function.
    %
    %  {SX} = FREE_SX(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(748, self, varargin{:});
    end
    function varargout = free_mx(self,varargin)
    %FREE_MX Get all the free variables of the function.
    %
    %  {MX} = FREE_MX(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(749, self, varargin{:});
    end
    function varargout = generate_lifted(self,varargin)
    %GENERATE_LIFTED Extract the functions needed for the Lifted Newton method.
    %
    %  [Function OUTPUT, Function OUTPUT] = GENERATE_LIFTED(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(750, self, varargin{:});
    end
    function varargout = n_nodes(self,varargin)
    %N_NODES Number of nodes in the algorithm.
    %
    %  int = N_NODES(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(751, self, varargin{:});
    end
    function varargout = n_instructions(self,varargin)
    %N_INSTRUCTIONS Number of instruction in the algorithm (SXFunction/MXFunction)
    %
    %  int = N_INSTRUCTIONS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(752, self, varargin{:});
    end
    function varargout = instruction_id(self,varargin)
    %INSTRUCTION_ID Identifier index of the instruction (SXFunction/MXFunction)
    %
    %  int = INSTRUCTION_ID(self, int k)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(753, self, varargin{:});
    end
    function varargout = instruction_input(self,varargin)
    %INSTRUCTION_INPUT Locations in the work vector for the inputs of the instruction
    %
    %  [int] = INSTRUCTION_INPUT(self, int k)
    %
    %(SXFunction/MXFunction)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(754, self, varargin{:});
    end
    function varargout = instruction_constant(self,varargin)
    %INSTRUCTION_CONSTANT Get the floating point output argument of an instruction ( SXFunction)
    %
    %  double = INSTRUCTION_CONSTANT(self, int k)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(755, self, varargin{:});
    end
    function varargout = instruction_output(self,varargin)
    %INSTRUCTION_OUTPUT Location in the work vector for the output of the instruction
    %
    %  [int] = INSTRUCTION_OUTPUT(self, int k)
    %
    %(SXFunction/MXFunction)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(756, self, varargin{:});
    end
    function varargout = instruction_MX(self,varargin)
    %INSTRUCTION_MX Get the MX node corresponding to an instruction ( MXFunction)
    %
    %  MX = INSTRUCTION_MX(self, int k)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(757, self, varargin{:});
    end
    function varargout = instructions_sx(self,varargin)
    %INSTRUCTIONS_SX Get the SX node corresponding to all instructions ( SXFunction)
    %
    %  SX = INSTRUCTIONS_SX(self)
    %
    %
    %Note: input and output instructions have no SX representation. This method
    %returns nan for those instructions.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(758, self, varargin{:});
    end
    function varargout = has_spfwd(self,varargin)
    %HAS_SPFWD Is the class able to propagate seeds through the algorithm?
    %
    %  bool = HAS_SPFWD(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(759, self, varargin{:});
    end
    function varargout = has_sprev(self,varargin)
    %HAS_SPREV Is the class able to propagate seeds through the algorithm?
    %
    %  bool = HAS_SPREV(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(760, self, varargin{:});
    end
    function varargout = sz_arg(self,varargin)
    %SZ_ARG Get required length of arg field.
    %
    %  size_t = SZ_ARG(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(761, self, varargin{:});
    end
    function varargout = sz_res(self,varargin)
    %SZ_RES Get required length of res field.
    %
    %  size_t = SZ_RES(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(762, self, varargin{:});
    end
    function varargout = sz_iw(self,varargin)
    %SZ_IW Get required length of iw field.
    %
    %  size_t = SZ_IW(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(763, self, varargin{:});
    end
    function varargout = sz_w(self,varargin)
    %SZ_W Get required length of w field.
    %
    %  size_t = SZ_W(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(764, self, varargin{:});
    end
    function varargout = name(self,varargin)
    %NAME Name of the function.
    %
    %  char = NAME(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(765, self, varargin{:});
    end
    function varargout = is_a(self,varargin)
    %IS_A Check if the function is of a particular type Optionally check if name
    %
    %  bool = IS_A(self, char type, bool recursive)
    %
    %matches one of the base classes (default true)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(766, self, varargin{:});
    end
    function varargout = assert_size_in(self,varargin)
    %ASSERT_SIZE_IN Assert that an input dimension is equal so some given value.
    %
    %  ASSERT_SIZE_IN(self, int i, int nrow, int ncol)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(771, self, varargin{:});
    end
    function varargout = assert_size_out(self,varargin)
    %ASSERT_SIZE_OUT Assert that an output dimension is equal so some given value.
    %
    %  ASSERT_SIZE_OUT(self, int i, int nrow, int ncol)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(772, self, varargin{:});
    end
    function varargout = checkout(self,varargin)
    %CHECKOUT Checkout a memory object.
    %
    %  int = CHECKOUT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(773, self, varargin{:});
    end
    function varargout = release(self,varargin)
    %RELEASE Release a memory object.
    %
    %  RELEASE(self, int mem)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(774, self, varargin{:});
    end
    function varargout = get_function(self,varargin)
    %GET_FUNCTION 
    %
    %  {char} = GET_FUNCTION(self)
    %  Function = GET_FUNCTION(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(775, self, varargin{:});
    end
    function varargout = has_function(self,varargin)
    %HAS_FUNCTION 
    %
    %  bool = HAS_FUNCTION(self, char fname)
    %
    %
      [varargout{1:nargout}] = casadiMEX(776, self, varargin{:});
    end
    function varargout = info(self,varargin)
    %INFO Obtain information about function
    %
    %  struct = INFO(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(777, self, varargin{:});
    end

     function s = saveobj(obj)
        try
            s.serialization = obj.serialize();
        catch exception
            warning(['Serializing of CasADi Function failed:' getReport(exception) ]);
            s = struct;
        end
     end
  
    function varargout = subsref(self,s)
      if numel(s)==1 && strcmp(s.type,'()')
        [varargout{1:nargout}]= paren(self, s.subs{:});
      else
        [varargout{1:nargout}] = builtin('subsref',self,s);
      end
   end
   function varargout = paren(self, varargin)
      if nargin==1 || (nargin>=2 && ischar(varargin{1}))
        % Named inputs: return struct
        assert(nargout<2, 'Syntax error');
        assert(mod(nargin,2)==1, 'Syntax error');
        arg = struct;
        for i=1:2:nargin-1
          assert(ischar(varargin{i}), 'Syntax error');
          arg.(varargin{i}) = varargin{i+1};
        end
        res = self.call(arg);
        varargout{1} = res;
      else
        % Ordered inputs: return variable number of outputs
        res = self.call(varargin);
        assert(nargout<=numel(res), 'Too many outputs');
        for i=1:max(min(1,numel(res)),nargout)
          varargout{i} = res{i};
        end
      end
    end
      function self = Function(varargin)
    %FUNCTION 
    %
    %  new_obj = FUNCTION()
    %  new_obj = FUNCTION(char fname)
    %  new_obj = FUNCTION(char name, {SX} ex_in, {SX} ex_out, struct opts)
    %  new_obj = FUNCTION(char name, {MX} ex_in, {MX} ex_out, struct opts)
    %  new_obj = FUNCTION(char name, struct:SX dict, {char} name_in, {char} name_out, struct opts)
    %  new_obj = FUNCTION(char name, struct:MX dict, {char} name_in, {char} name_out, struct opts)
    %  new_obj = FUNCTION(char name, {SX} ex_in, {SX} ex_out, {char} name_in, {char} name_out, struct opts)
    %  new_obj = FUNCTION(char name, {MX} ex_in, {MX} ex_out, {char} name_in, {char} name_out, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION()
    %
    %
    %
    %Default constructor, null pointer.
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char fname)
    %
    %
    %
    %Construct from a file.
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char name, {SX} ex_in, {SX} ex_out, struct opts)
    %  FUNCTION(char name, struct:SX dict, {char} name_in, {char} name_out, struct opts)
    %  FUNCTION(char name, {SX} ex_in, {SX} ex_out, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %Construct an SX function.
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char name, {MX} ex_in, {MX} ex_out, struct opts)
    %  FUNCTION(char name, struct:MX dict, {char} name_in, {char} name_out, struct opts)
    %  FUNCTION(char name, {MX} ex_in, {MX} ex_out, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %Construct an MX function.
    %
    %
    %
    %.............
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(778, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(676, varargin{:});
    end
    function varargout = jit(varargin)
    %JIT To resolve ambiguity on some compilers.
    %
    %  Function = JIT(char name, char body, {char} name_in, {char} name_out, struct opts)
    %  Function = JIT(char name, char body, {char} name_in, {char} name_out, {Sparsity} sparsity_in, {Sparsity} sparsity_out, struct opts)
    %
    %
    %Create a just-in-time compiled function from a C language string The names
    %and sparsity patterns of all the inputs and outputs must be provided. If
    %sparsities are not provided, all inputs and outputs are assumed to be
    %scalar. Only specify the function body, assuming that input and output
    %nonzeros are stored in arrays with the specified naming convension. The data
    %type used is 'casadi_real', which is typically equal to 'double` or another
    %data type with the same API as 'double'.
    %
    %Inputs may be null pointers. This means that the all entries are zero.
    %Outputs may be null points. This means that the corresponding result can be
    %ignored.
    %
    %If an error occurs in the evaluation, issue "return 1;";
    %
    %The final generated function will have a structure similar to:
    %
    %casadi_int fname(const casadi_real** arg, casadi_real** res, casadi_int* iw,
    %casadi_real* w, void* mem) { const casadi_real *x1, *x2; casadi_real *r1,
    %*r2; x1 = *arg++; x2 = *arg++; r1 = *res++; r2 = *res++; <FUNCTION_BODY>
    %return 0; }
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(677, varargin{:});
    end
    function varargout = conditional(varargin)
    %CONDITIONAL 
    %
    %  Function = CONDITIONAL(char name, {Function} f, Function f_def, struct opts)
    %
    %
     [varargout{1:nargout}] = casadiMEX(722, varargin{:});
    end
    function varargout = bspline(varargin)
    %BSPLINE 
    %
    %  Function = BSPLINE(char name, {[double]} knots, [double] coeffs, [int] degree, int m, struct opts)
    %
    %
     [varargout{1:nargout}] = casadiMEX(723, varargin{:});
    end
    function varargout = if_else(varargin)
    %IF_ELSE 
    %
    %  Function = IF_ELSE(char name, Function f_true, Function f_false, struct opts)
    %
    %
     [varargout{1:nargout}] = casadiMEX(724, varargin{:});
    end
    function varargout = check_name(varargin)
    %CHECK_NAME 
    %
    %  bool = CHECK_NAME(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(767, varargin{:});
    end
    function varargout = fix_name(varargin)
    %FIX_NAME 
    %
    %  char = FIX_NAME(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(768, varargin{:});
    end
    function varargout = load(varargin)
    %LOAD 
    %
    %  Function = LOAD(char filename)
    %
    %
     [varargout{1:nargout}] = casadiMEX(769, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  Function = DESERIALIZE(std::istream & stream)
    %  Function = DESERIALIZE(casadi::DeserializingStream & s)
    %  Function = DESERIALIZE(char s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(770, varargin{:});
    end

     function obj = loadobj(s)
        try
          if isstruct(s)
             obj = casadi.Function.deserialize(s.serialization);
          else
             obj = s;
          end
        catch exception
            warning(['Serializing of CasADi Function failed:' getReport(exception) ]);
            s = struct;
        end
     end
    end
end
