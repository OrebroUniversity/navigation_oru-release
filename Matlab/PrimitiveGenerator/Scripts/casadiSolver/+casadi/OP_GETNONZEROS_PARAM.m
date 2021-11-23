function v = OP_GETNONZEROS_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 81);
  end
  v = vInitialized;
end
