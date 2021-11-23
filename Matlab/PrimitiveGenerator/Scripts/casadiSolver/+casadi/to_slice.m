function varargout = to_slice(varargin)
    %TO_SLICE Construct from an index vector (requires is_slice(v) to be true)
    %
    %  Slice = TO_SLICE([int] v, bool ind1)
    %
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(201, varargin{:});
end
