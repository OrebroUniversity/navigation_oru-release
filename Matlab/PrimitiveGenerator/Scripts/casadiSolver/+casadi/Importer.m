classdef  Importer < casadi.SharedObject & casadi.PrintableCommon
    %IMPORTER Importer.
    %
    %
    %
    %Just-in-time compilation of code
    %
    %General information
    %===================
    %
    %
    %
    %List of plugins
    %===============
    %
    %
    %
    %- clang
    %
    %- shell
    %
    %Note: some of the plugins in this list might not be available on your
    %system. Also, there might be extra plugins available to you that are not
    %listed here. You can obtain their documentation with
    %Importer.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %clang
    %-----
    %
    %
    %
    %Interface to the JIT compiler CLANG
    %
    %>List of available options
    %
    %+--------------+-----------------+-----------------------------------------+
    %|      Id      |      Type       |               Description               |
    %+==============+=================+=========================================+
    %| flags        | OT_STRINGVECTOR | Compile flags for the JIT compiler.     |
    %|              |                 | Default: None                           |
    %+--------------+-----------------+-----------------------------------------+
    %| include_path | OT_STRING       | Include paths for the JIT compiler. The |
    %|              |                 | include directory shipped with CasADi   |
    %|              |                 | will be automatically appended.         |
    %+--------------+-----------------+-----------------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %shell
    %-----
    %
    %
    %
    %Interface to the JIT compiler SHELL
    %
    %>List of available options
    %
    %+----------------------+-----------------+---------------------------------+
    %|          Id          |      Type       |           Description           |
    %+======================+=================+=================================+
    %| cleanup              | OT_BOOL         | Cleanup temporary files when    |
    %|                      |                 | unloading. Default: true        |
    %+----------------------+-----------------+---------------------------------+
    %| compiler             | OT_STRING       | Compiler command                |
    %+----------------------+-----------------+---------------------------------+
    %| compiler_flags       | OT_STRINGVECTOR | Alias for 'compiler_flags'      |
    %+----------------------+-----------------+---------------------------------+
    %| compiler_output_flag | OT_STRING       | Compiler flag to denote object  |
    %|                      |                 | output. Default: '-o '          |
    %+----------------------+-----------------+---------------------------------+
    %| compiler_setup       | OT_STRING       | Compiler setup command.         |
    %|                      |                 | Intended to be fixed. The       |
    %|                      |                 | 'flag' option is the prefered   |
    %|                      |                 | way to set custom flags.        |
    %+----------------------+-----------------+---------------------------------+
    %| extra_suffixes       | OT_STRINGVECTOR | List of suffixes for extra      |
    %|                      |                 | files that the compiler may     |
    %|                      |                 | generate. Default: None         |
    %+----------------------+-----------------+---------------------------------+
    %| flags                | OT_STRINGVECTOR | Compile flags for the JIT       |
    %|                      |                 | compiler. Default: None         |
    %+----------------------+-----------------+---------------------------------+
    %| folder               | OT_STRING       | Folder to put temporary objects |
    %|                      |                 | in.                             |
    %+----------------------+-----------------+---------------------------------+
    %| linker               | OT_STRING       | Linker command                  |
    %+----------------------+-----------------+---------------------------------+
    %| linker_flags         | OT_STRINGVECTOR | Linker flags for the JIT        |
    %|                      |                 | compiler. Default: None         |
    %+----------------------+-----------------+---------------------------------+
    %| linker_output_flag   | OT_STRING       | Linker flag to denote shared    |
    %|                      |                 | library output. Default: '-o '  |
    %+----------------------+-----------------+---------------------------------+
    %| linker_setup         | OT_STRING       | Linker setup command. Intended  |
    %|                      |                 | to be fixed. The 'flag' option  |
    %|                      |                 | is the prefered way to set      |
    %|                      |                 | custom flags.                   |
    %+----------------------+-----------------+---------------------------------+
    %| name                 | OT_STRING       | The file name used to write out |
    %|                      |                 | compiled objects/libraries. The |
    %|                      |                 | actual file names used depend   |
    %|                      |                 | on 'temp_suffix' and include    |
    %|                      |                 | extensions. Default:            |
    %|                      |                 | 'tmp_casadi_compiler_shell'     |
    %+----------------------+-----------------+---------------------------------+
    %| temp_suffix          | OT_BOOL         | Use a temporary (seemingly      |
    %|                      |                 | random) filename suffix for     |
    %|                      |                 | file names. This is desired for |
    %|                      |                 | thread-safety. This behaviour   |
    %|                      |                 | may defeat caching compiler     |
    %|                      |                 | wrappers. Default: true         |
    %+----------------------+-----------------+---------------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %Joris Gillis
    %
    %C++ includes: importer.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = plugin_name(self,varargin)
    %PLUGIN_NAME Query plugin name.
    %
    %  char = PLUGIN_NAME(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(880, self, varargin{:});
    end
    function varargout = has_function(self,varargin)
    %HAS_FUNCTION 
    %
    %  bool = HAS_FUNCTION(self, char symname)
    %
    %
      [varargout{1:nargout}] = casadiMEX(881, self, varargin{:});
    end
    function varargout = has_meta(self,varargin)
    %HAS_META Does a meta entry exist?
    %
    %  bool = HAS_META(self, char cmd, int ind)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(882, self, varargin{:});
    end
    function varargout = get_meta(self,varargin)
    %GET_META Get entry as a text.
    %
    %  char = GET_META(self, char cmd, int ind)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(883, self, varargin{:});
    end
    function varargout = inlined(self,varargin)
    %INLINED Check if a function is inlined.
    %
    %  bool = INLINED(self, char symname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(884, self, varargin{:});
    end
    function varargout = body(self,varargin)
    %BODY Get the function body, if inlined.
    %
    %  char = BODY(self, char symname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(885, self, varargin{:});
    end
    function varargout = library(self,varargin)
    %LIBRARY Get library name.
    %
    %  char = LIBRARY(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(886, self, varargin{:});
    end
    function varargout = serialize(self,varargin)
    %SERIALIZE Serialize an object.
    %
    %  SERIALIZE(self, casadi::SerializingStream & s)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(887, self, varargin{:});
    end
    function self = Importer(varargin)
    %IMPORTER 
    %
    %  new_obj = IMPORTER()
    %  new_obj = IMPORTER(char name, char compiler, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  IMPORTER(char name, char compiler, struct opts)
    %
    %
    %
    %Importer factory.
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
    %  IMPORTER()
    %
    %
    %
    %Default constructor.
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
        tmp = casadiMEX(889, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(890, self);
          self.SwigClear();
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
     [varargout{1:nargout}] = casadiMEX(875, varargin{:});
    end
    function varargout = test_cast(varargin)
    %TEST_CAST 
    %
    %  bool = TEST_CAST(casadi::SharedObjectInternal const * ptr)
    %
    %
     [varargout{1:nargout}] = casadiMEX(876, varargin{:});
    end
    function varargout = has_plugin(varargin)
    %HAS_PLUGIN 
    %
    %  bool = HAS_PLUGIN(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(877, varargin{:});
    end
    function varargout = load_plugin(varargin)
    %LOAD_PLUGIN 
    %
    %  LOAD_PLUGIN(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(878, varargin{:});
    end
    function varargout = doc(varargin)
    %DOC 
    %
    %  char = DOC(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(879, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  Importer = DESERIALIZE(casadi::DeserializingStream & s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(888, varargin{:});
    end
  end
end
