function varargout = lookupvector(varargin)
    %LOOKUPVECTOR 
    %
    %  [int] = LOOKUPVECTOR([int] v)
    %  [int] = LOOKUPVECTOR([int] v, int size)
    %
    %
    %.......
    %
    %::
    %
    %  LOOKUPVECTOR([int] v)
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
    %  LOOKUPVECTOR([int] v, int size)
    %
    %
    %
    %Returns a vector for quickly looking up entries of supplied list.
    %
    %lookupvector[i]!=-1 <=> v contains i v[lookupvector[i]] == i <=> v contains
    %i
    %
    %Duplicates are treated by looking up last occurrence
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(36, varargin{:});
end
