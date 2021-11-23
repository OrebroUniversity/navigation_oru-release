function v = OP_LT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 30);
  end
  v = vInitialized;
end
