function v = OP_FMOD()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 39);
  end
  v = vInitialized;
end
