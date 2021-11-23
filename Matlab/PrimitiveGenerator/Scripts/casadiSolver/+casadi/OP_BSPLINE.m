function v = OP_BSPLINE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 101);
  end
  v = vInitialized;
end
