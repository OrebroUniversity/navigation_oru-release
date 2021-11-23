function v = OP_SETNONZEROS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 84);
  end
  v = vInitialized;
end
