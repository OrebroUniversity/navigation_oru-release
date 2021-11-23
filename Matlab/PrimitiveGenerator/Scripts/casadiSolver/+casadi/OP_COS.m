function v = OP_COS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 25);
  end
  v = vInitialized;
end
