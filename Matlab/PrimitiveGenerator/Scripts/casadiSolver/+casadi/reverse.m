function varargout = reverse(varargin)
    %REVERSE Reverse directional derivative.
    %
    %  {{DM}} = REVERSE({DM} ex, {DM} arg, {{DM}} v, struct opts)
    %  {{SX}} = REVERSE({SX} ex, {SX} arg, {{SX}} v, struct opts)
    %  {{MX}} = REVERSE({MX} ex, {MX} arg, {{MX}} v, struct opts)
    %
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(866, varargin{:});
end
