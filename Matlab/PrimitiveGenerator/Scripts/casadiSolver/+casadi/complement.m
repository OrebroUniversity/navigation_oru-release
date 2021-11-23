function varargout = complement(varargin)
    %COMPLEMENT Returns the list of all i in [0, size[ not found in supplied list.
    %
    %  [int] = COMPLEMENT([int] v, int size)
    %
    %
    %The supplied vector may contain duplicates and may be non-monotonous The
    %supplied vector will be checked for bounds The result vector is guaranteed
    %to be monotonously increasing
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(35, varargin{:});
end
