function v = OP_LIFT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 99);
  end
  v = vInitialized;
end
