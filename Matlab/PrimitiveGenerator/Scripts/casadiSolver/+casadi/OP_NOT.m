function v = OP_NOT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 34);
  end
  v = vInitialized;
end
