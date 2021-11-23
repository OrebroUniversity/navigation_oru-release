function v = OP_FMAX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 46);
  end
  v = vInitialized;
end
