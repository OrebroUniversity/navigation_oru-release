function varargout = external(varargin)
    %EXTERNAL Load a just-in-time compiled external function File name given.
    %
    %  Function = EXTERNAL(char name, struct opts)
    %  Function = EXTERNAL(char name, Importer li, struct opts)
    %  Function = EXTERNAL(char name, char bin_name, struct opts)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  EXTERNAL(char name, Importer li, struct opts)
    %
    %
    %
    %Load a just-in-time compiled external function File name given.
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
    %  EXTERNAL(char name, struct opts)
    %
    %
    %
    %Load an external function from a shared library.
    %
    %Parameters:
    %-----------
    %
    %name:  Name as in the label assigned to a CasADi Function object:
    %Function(name,...,...) Will be used to look up symbols/functions named eg.
    %<name>_eval Use nm (linux/osx) or depends.exe (win) to check which symbols
    %are present in your shared library
    %
    %File name is assumed to be ./<name>.so
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
    %  EXTERNAL(char name, char bin_name, struct opts)
    %
    %
    %
    %Load an external function from a shared library.
    %
    %Parameters:
    %-----------
    %
    %name:  Name as in the label assigned to a CasADi Function object:
    %Function(name,...,...) Will be used to look up symbols/functions named eg.
    %<name>_eval Use nm (linux/osx) or depends.exe (win) to check which symbols
    %are present in your shared library
    %
    %bin_name:  File name of the shared library
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(779, varargin{:});
end
