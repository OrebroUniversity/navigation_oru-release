function v = OP_FLOOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 37);
  end
  v = vInitialized;
end
