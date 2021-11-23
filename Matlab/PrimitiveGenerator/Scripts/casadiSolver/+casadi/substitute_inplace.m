function varargout = substitute_inplace(varargin)
    %SUBSTITUTE_INPLACE Inplace substitution with piggyback expressions Substitute variables v out
    %
    %  [{DM} INOUT1, {DM} INOUT2] = SUBSTITUTE_INPLACE({DM} v, bool reverse)
    %  [{SX} INOUT1, {SX} INOUT2] = SUBSTITUTE_INPLACE({SX} v, bool reverse)
    %  [{MX} INOUT1, {MX} INOUT2] = SUBSTITUTE_INPLACE({MX} v, bool reverse)
    %
    %of the expressions vdef sequentially, as well as out of a number of other
    %expressions piggyback.
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(868, varargin{:});
end
