function varargout = graph_substitute(varargin)
    %GRAPH_SUBSTITUTE Substitute multiple expressions in graph Substitute variable var with
    %
    %  MX = GRAPH_SUBSTITUTE(MX ex, {MX} v, {MX} vdef)
    %  {MX} = GRAPH_SUBSTITUTE({MX} ex, {MX} v, {MX} vdef)
    %
    %expression expr in multiple expressions, preserving nodes.
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(872, varargin{:});
end
