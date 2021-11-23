function v = OP_VERTSPLIT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 75);
  end
  v = vInitialized;
end
