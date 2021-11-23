function v = OP_NORM1()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 90);
  end
  v = vInitialized;
end
