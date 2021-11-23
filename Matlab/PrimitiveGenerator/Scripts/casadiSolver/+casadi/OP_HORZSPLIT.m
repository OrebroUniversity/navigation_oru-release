function v = OP_HORZSPLIT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 74);
  end
  v = vInitialized;
end
