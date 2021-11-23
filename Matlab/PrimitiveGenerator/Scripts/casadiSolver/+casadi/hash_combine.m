function varargout = hash_combine(varargin)
    %HASH_COMBINE Generate a hash value incrementally (function taken from boost)
    %
    %  HASH_COMBINE(std::size_t & seed, [int] v)
    %  HASH_COMBINE(std::size_t & seed, casadi_int const * v, std::size_t sz)
    %
    %
    %
    %
    %.......
    %
    %::
    %
    %  HASH_COMBINE(std::size_t & seed, [int] v)
    %
    %
    %
    %Generate a hash value incrementally (function taken from boost)
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
    %  HASH_COMBINE(std::size_t & seed, casadi_int const * v, std::size_t sz)
    %
    %
    %
    %Generate a hash value incrementally, array.
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(178, varargin{:});
end
