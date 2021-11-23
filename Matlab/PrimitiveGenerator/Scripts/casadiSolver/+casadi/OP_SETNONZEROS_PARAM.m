function v = OP_SETNONZEROS_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 85);
  end
  v = vInitialized;
end
