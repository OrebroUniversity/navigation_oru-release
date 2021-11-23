function v = OP_LOG()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 18);
  end
  v = vInitialized;
end
