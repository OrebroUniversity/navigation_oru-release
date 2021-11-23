function v = OP_SUB()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 13);
  end
  v = vInitialized;
end
