function v = OP_MONITOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 88);
  end
  v = vInitialized;
end
