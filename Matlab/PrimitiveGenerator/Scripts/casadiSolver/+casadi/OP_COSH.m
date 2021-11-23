function v = OP_COSH()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 49);
  end
  v = vInitialized;
end
