function v = OP_INVERSE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 67);
  end
  v = vInitialized;
end
