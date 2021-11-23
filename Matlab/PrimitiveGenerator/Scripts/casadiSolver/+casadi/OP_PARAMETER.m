function v = OP_PARAMETER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 58);
  end
  v = vInitialized;
end
