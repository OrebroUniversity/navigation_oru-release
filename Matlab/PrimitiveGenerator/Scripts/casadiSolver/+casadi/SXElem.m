classdef  SXElem < SwigRef
    %SXELEM 
    %
    %   = SXELEM()
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function self = SXElem(varargin)
    %SXELEM 
    %
    %  new_obj = SXELEM()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(512, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(513, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
